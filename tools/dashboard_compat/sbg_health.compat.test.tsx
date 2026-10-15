// @vitest-environment jsdom
//
// Runs INSIDE a dashboard checkout (tools/check_dashboard_health.sh copies it there together
// with the driver's serialized fixture) and exercises the dashboard's own ingestion path —
// CDR decoder, HealthDiscovery, verdict/issue logic and the Home health widget — against the
// DiagnosticArrays this driver really serializes. Nothing here re-implements the consumer.
import { cleanup } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import type { HealthWidgetConfig } from "../../config/schema";
import { HealthWidget } from "../../home/widgets/HealthWidget";
import { buildGraph } from "../../ros/graph";
import { buildRos2StaticDecoder } from "../../schema/decoders/staticDefs";
import { renderWith } from "../../test/harness";
import type { Transport } from "../../transport/types";
import { HealthDiscovery } from "../discovery";
import { healthIssues, observationText } from "../summary";
import { healthVerdict, statusCurrent, type HealthService, type HealthStatus } from "../types";
import fixtures from "./health_diagnostics.json";

type Report = keyof typeof fixtures;

// The publisher as rmw_zenoh advertises it. The node runs in ROS namespace /front on purpose:
// the fleet identity below must come from the health.* values, not from this.
const topic = buildGraph([
  "@ros2_lv/0/z/1/2/MP/%/%front/sbg_driver/%diagnostics/diagnostic_msgs::msg::dds_::DiagnosticArray_/hash/::,10::::",
]).topics[0];
const KEY = "fleet/uav6/svc/ins/health";
const bytes = (s: string) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
const transport = () =>
  ({ liveliness: { subscribe: vi.fn(async () => ({ close: vi.fn() })) }, subscribe: vi.fn(async () => ({ close: vi.fn() })) }) as unknown as Transport;
const widget = (over: Partial<HealthWidgetConfig> = {}): HealthWidgetConfig => ({ type: "health", history_s: 300, precision: 1, ...over });

afterEach(() => { cleanup(); localStorage.clear(); });

/** Decode the named reports, in order, through the real decoder into a fresh store. */
async function ingest(...reports: Report[]) {
  const decoder = await buildRos2StaticDecoder("diagnostic_msgs/msg/DiagnosticArray");
  const discovery = new HealthDiscovery(transport());
  let services: HealthService[] = [];
  await discovery.start((s) => { services = s; });
  for (const report of reports) {
    discovery.ingestRos(topic, decoder.decode(bytes(fixtures[report])));
    expect(decoder.lastWarning?.(), report).toBeUndefined();
  }
  expect(services, "one reporter per instance").toHaveLength(1);
  const service = services[0];
  const status = (component: string) => service.snapshot.status.find((s) => s.name === `ins: ${component}`);
  const must = (component: string): HealthStatus => {
    const found = status(component);
    if (!found) throw new Error(`no component ${component}`);
    return found;
  };
  return { discovery, service, services, status, must };
}

const instance = () => document.querySelector<HTMLElement>('.health-summary[data-instance="ins"]')!;
const verdictText = () => instance().querySelector(".health-verdict")?.textContent;
const metrics = () => [...instance().querySelectorAll(".health-summary-metric")]
  .map((r) => [r.querySelector(".health-metric-label")?.textContent, r.querySelector(".row-value")?.textContent]);
const pills = () => [...instance().querySelectorAll(".health-issue")]
  .map((p) => `${p.querySelector(".health-issue-severity")?.textContent} ${p.querySelector(".health-issue-label")?.textContent}`);

describe("sbg_driver /diagnostics through the dashboard", () => {
  it("decodes every report and attributes it to the fleet instance, not the ROS namespace", async () => {
    for (const report of Object.keys(fixtures) as Report[]) {
      const { service } = await ingest(report);
      expect(service.key, report).toBe(KEY);
      expect(service.vehicleId).toBe("uav6");
      expect(service.instance).toBe("ins");
      expect(service.source).toBe("ros2");
      expect(service.snapshot.service).toBe("sbg");
      for (const s of service.snapshot.status) {
        expect(s.name.startsWith("ins: "), s.name).toBe(true);
        expect(typeof s.values["health.sequence"], s.name).toBe("number");
        expect(typeof s.values["health.sample_age_s"]).toBe("number");
        expect(typeof s.values["health.stale_after_s"]).toBe("number");
        expect(typeof s.values["health.publish_interval_s"]).toBe("number");
        expect(typeof s.values["health.publisher_id"]).toBe("string");
        expect(typeof s.values["health.sample_id"]).toBe("string");
      }
    }
  });

  it("standby before any observation is healthy and raises nothing", async () => {
    const { service, services, must } = await ingest("standby_before_activation");
    expect(must("service").values.state).toBe("standby");
    expect(healthVerdict(service, service.snapshot.status, Date.now())).toMatchObject({ text: "OK", level: "ok" });
    expect(healthIssues(service, service.snapshot.status, Date.now())).toEqual([]);
    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(verdictText()).toBe("OK");
    expect(pills()).toEqual([]);
    expect(metrics()).toEqual([]); // no temperature yet, and no invented one
  });

  it("active: temperature, hardware, RTK grade and exactly one pill for the one active fault", async () => {
    const { discovery, service, services, must, status } = await ingest("standby_before_activation", "active");
    const now = Date.now();
    expect(healthVerdict(service, service.snapshot.status, now)).toMatchObject({ text: "ERROR", level: "err" });

    const temperature = must("temperature");
    expect(temperature.values["temp.imu_c"]).toBe(41.5);
    expect(temperature.hardware_id).toBe("QUANTA-USG 000123");
    expect(statusCurrent(service, temperature, now)).toBe(true);
    expect(observationText(service, temperature, now)).toMatch(/^current · sample 0s old/);
    expect(discovery.series(KEY, "ins: temperature", "temp.imu_c")).toHaveLength(1);

    expect(must("alert/MAIN_POWER")).toMatchObject({ level: 2, message: "main power supply fault" });
    expect(must("alert/MAIN_POWER").values).toMatchObject({
      "alert.code": "MAIN_POWER", "alert.category": "power", "alert.active": true, "alert.severity": "ERROR" });
    expect(must("alert/CPU")).toMatchObject({ level: 0, message: "cleared" }); // explicit clear, retained
    expect(must("alert/CPU").values["alert.active"]).toBe(false);
    expect(must("sensor alerts").values["alerts.active"]).toBe(1);
    expect(must("gnss").values).toMatchObject({ "gnss.rtk": "fixed", "gnss.fix_type": "RTK_INT" });
    expect(must("gnss").message).toBe("RTK fixed");
    expect(must("ekf").values["ekf.solution_mode"]).toBe("NAV_POSITION");
    expect(must("gnss heading").values["gnss.heading_valid"]).toBe("true");
    expect(must("service").hardware_id).toBe(""); // software component
    expect(status("power")).toBeUndefined();
    for (const s of service.snapshot.status) for (const key of ["supply.voltage_v", "supply.current_a", "supply.power_w"]) {
      expect(key in s.values, `${s.name} ${key}`).toBe(false);
    }
    expect(must("device").values["health.metric.supply.power_w.state"]).toBe("unsupported");
    expect(discovery.events.some((e) => e.name === "ins: alert/MAIN_POWER" && e.level === 2)).toBe(true);

    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(verdictText()).toBe("ERROR");
    expect(metrics()).toEqual([["imu", "41.5 °C"]]);
    // The alert summary repeats the per-code fault and the cleared alert is omitted.
    expect(pills()).toEqual(["ERROR MAIN_POWER"]);
  });

  it("a silent link reads as stale without losing the fault's severity or inventing fresh samples", async () => {
    const { discovery, service, services, must } = await ingest("active", "silent");
    const now = Date.now();
    expect(healthVerdict(service, service.snapshot.status, now)).toMatchObject({ text: "ERROR · STALE", level: "err" });
    expect(must("stream")).toMatchObject({ level: 2 });
    const alert = must("alert/MAIN_POWER");
    expect(alert.level).toBe(3);
    expect(alert.values["health.last_level"]).toBe(2);
    const issues = healthIssues(service, service.snapshot.status, now);
    expect(issues.find((i) => i.label === "MAIN_POWER")).toMatchObject({ severity: "ERROR", color: "err" });
    expect(issues.find((i) => i.label === "stream")).toMatchObject({ severity: "ERROR", color: "err" });

    const temperature = must("temperature");
    expect(temperature.values["temp.imu_c"]).toBe(41.5); // last known, kept
    expect(temperature.values["health.availability"]).toBe("stale");
    expect(statusCurrent(service, temperature, now)).toBe(false);
    expect(observationText(service, temperature, now)).toMatch(/^Stale · last known · sample 6s old/);
    // The cached reading was republished; it must not become a second trend point.
    expect(discovery.series(KEY, "ins: temperature", "temp.imu_c")).toHaveLength(1);

    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(verdictText()).toBe("ERROR · STALE");
    expect(pills()).toContain("ERROR MAIN_POWER");
    expect(pills()).toContain("ERROR stream");
    expect(instance().querySelector(".health-metric-quality")?.textContent).toBe("last known");
  });

  it("confirmed standby keeps the last known fault but raises no staleness", async () => {
    const { service, services, must } = await ingest("active", "silent", "standby");
    expect(must("service")).toMatchObject({ level: 0, message: "standby" });
    expect(must("temperature").values["health.availability"]).toBe("paused");
    expect(healthVerdict(service, service.snapshot.status, Date.now())).toMatchObject({ text: "ERROR", level: "err" });
    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(pills()).toEqual(["ERROR MAIN_POWER"]);
  });

  it("a restarted publisher starts a new inventory; the old boot cannot come back", async () => {
    const { service, services, status, must, discovery } = await ingest("active", "restart", "silent");
    expect(status("alert/MAIN_POWER")).toBeUndefined();
    expect(status("gnss heading")).toBeUndefined();
    expect(must("stream")).toMatchObject({ level: 0 }); // `silent` (old boot) was ignored
    expect(must("temperature").values["temp.imu_c"]).toBe(0); // a valid zero
    expect(discovery.events.some((e) => e.name === "ins: alert/MAIN_POWER")).toBe(true); // journal keeps it

    expect(healthVerdict(service, service.snapshot.status, Date.now())).toMatchObject({ text: "WARN", level: "warn" });
    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(metrics()).toEqual([["imu", "0.0 °C"]]); // zero, not a dash
    expect(pills().sort()).toEqual(["WARN ekf", "WARN gnss"]);
  });

  it("a failed temperature read preserves the last measurement, and a configured limit travels as WARN", async () => {
    const lost = await ingest("restart", "temperature_unavailable");
    expect(lost.must("temperature").values["temp.imu_c"]).toBe(0);
    expect(lost.must("temperature").level).toBe(3);
    expect(lost.must("temperature").values["health.sample_age_s"]).toBe(1);
    expect(lost.must("temperature").values["health.last_level"]).toBe(0);
    expect(lost.must("temperature").values["health.metric.temp.imu_c.state"]).toBe("unavailable");
    expect(lost.must("device").level).toBe(0); // the device/power side is unaffected
    renderWith(<HealthWidget widget={widget()} />, { healths: lost.services });
    expect(metrics()).toEqual([["imu", "0.0 °C"]]);
    expect(instance().querySelector(".health-metric-quality")?.textContent).toBe("last known");
    expect(pills()).toEqual(["WARN temperature"]);
    cleanup();

    const hot = await ingest("restart", "temperature_unavailable", "temperature_limit");
    expect(hot.must("temperature")).toMatchObject({ level: 1 });
    expect(hot.must("temperature").message).toContain("warn_above 60");
    expect(hot.must("temperature").values["temp.imu_c"]).toBe(63.25);
    renderWith(<HealthWidget widget={widget()} />, { healths: hot.services });
    expect(verdictText()).toBe("WARN");
    expect(pills()).toEqual(["WARN temperature"]);
  });

  it("keeps a temperature ERROR and its measurement after an invalid read", async () => {
    const { discovery, service, services, must } = await ingest("restart", "temperature_error", "temperature_error_unavailable");
    expect(must("temperature").level).toBe(3);
    expect(must("temperature").values).toMatchObject({
      "health.last_level": 2, "health.availability": "stale", "temp.imu_c": 95,
      "health.sample_age_s": 1, "health.metric.temp.imu_c.state": "unavailable" });
    expect(healthVerdict(service, service.snapshot.status, Date.now())).toMatchObject({ text: "ERROR · STALE", level: "err" });
    expect(discovery.series(KEY, "ins: temperature", "temp.imu_c")).toHaveLength(2);
    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(metrics()).toEqual([["imu", "95.0 °C"]]);
    expect(pills()).toContain("ERROR temperature");
    expect(instance().querySelector(".health-metric-quality")?.textContent).toBe("last known");
  });

  it("is found by auto-discovery and by explicit vehicle/instance or bare instance references", async () => {
    const { services } = await ingest("restart");
    const shown = () => [...document.querySelectorAll(".health-summary")].map((n) =>
      `${n.getAttribute("data-instance")}:${n.querySelector(".pill")?.textContent}`);

    renderWith(<HealthWidget widget={widget()} />, { healths: services });
    expect(shown()).toEqual(["ins:WARN"]);
    cleanup();
    renderWith(<HealthWidget widget={widget({ services: ["uav6/ins"] })} />, { healths: services });
    expect(shown()).toEqual(["ins:WARN"]);
    cleanup();
    renderWith(<HealthWidget widget={widget({ services: ["ins"] })} />, { healths: services });
    expect(shown()).toEqual(["ins:WARN"]);
    cleanup();
    // Another vehicle's instance of the same name is a different reporter.
    renderWith(<HealthWidget widget={widget({ services: ["uav7/ins"] })} />, { healths: services });
    expect(shown()).toEqual(["uav7/ins:not reported"]);
  });
});
