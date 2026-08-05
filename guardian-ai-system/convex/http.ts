import { httpRouter } from "convex/server";
import { httpAction } from "./_generated/server";
import { api } from "./_generated/api";

const http = httpRouter();

// GET /api/device-settings?ownerId=...
http.route({
  path: "/api/device-settings",
  method: "GET",
  handler: httpAction(async (ctx, request) => {
    const url = new URL(request.url);
    const ownerId = url.searchParams.get("ownerId");
    if (!ownerId) {
      return new Response("Missing ownerId", { status: 400 });
    }

    try {
      const settings = await ctx.runQuery(api.deviceSettings.get, { ownerId });
      return new Response(JSON.stringify(settings), {
        status: 200,
        headers: {
          "Content-Type": "application/json",
          "Access-Control-Allow-Origin": "*",
        },
      });
    } catch (error: any) {
      return new Response(JSON.stringify({ error: error.message }), {
        status: 500,
        headers: { "Content-Type": "application/json" },
      });
    }
  }),
});

// POST /api/telemetry
http.route({
  path: "/api/telemetry",
  method: "POST",
  handler: httpAction(async (ctx, request) => {
    try {
      const body = await request.json();
      const { ownerId, temperature, humidity, weight, online, lastSeen, wifiRSSI, deviceStatus, rawScaleValue, calibrationFactor, clearPendingCommand, cameraStreamUrl, clearPendingFeedRequest } = body;

      if (!ownerId) {
        return new Response("Missing ownerId", { status: 400 });
      }

      await ctx.runMutation(api.deviceSettings.update, {
        ownerId,
        currentTemperature: temperature !== undefined ? parseFloat(temperature) : undefined,
        currentHumidity: humidity !== undefined ? parseFloat(humidity) : undefined,
        currentWeight: weight !== undefined ? parseFloat(weight) : undefined,
        online: online !== undefined ? !!online : undefined,
        lastSeen: lastSeen !== undefined ? parseFloat(lastSeen) : Date.now(),
        wifiRSSI: wifiRSSI !== undefined ? parseFloat(wifiRSSI) : undefined,
        deviceStatus: deviceStatus || undefined,
        rawScaleValue: rawScaleValue !== undefined ? parseFloat(rawScaleValue) : undefined,
        calibrationFactor: calibrationFactor !== undefined ? parseFloat(calibrationFactor) : undefined,
        pendingCommand: clearPendingCommand === true ? "" : (body.pendingCommand !== undefined ? body.pendingCommand : undefined),
        cameraStreamUrl: cameraStreamUrl !== undefined ? cameraStreamUrl : undefined,
        pendingFeedRequest: clearPendingFeedRequest === true ? false : (body.pendingFeedRequest !== undefined ? !!body.pendingFeedRequest : undefined),
      });

      return new Response(JSON.stringify({ success: true }), {
        status: 200,
        headers: {
          "Content-Type": "application/json",
          "Access-Control-Allow-Origin": "*",
        },
      });
    } catch (error: any) {
      return new Response(JSON.stringify({ error: error.message }), {
        status: 500,
        headers: { "Content-Type": "application/json" },
      });
    }
  }),
});

// POST /api/feed-event
http.route({
  path: "/api/feed-event",
  method: "POST",
  handler: httpAction(async (ctx, request) => {
    try {
      const body = await request.json();
      const { ownerId, amountDispensed, triggerSource, temperature, humidity } = body;

      if (!ownerId) {
        return new Response("Missing ownerId", { status: 400 });
      }

      // Map triggerSource ("PIR", "WEB", "MANUAL_BUTTON") to Convex expected values
      let convexTrigger: "PIR" | "Button" | "Web" | "Bluetooth" = "Button";
      let feedingMethod: "Manual" | "Automatic" = "Manual";

      if (triggerSource === "PIR") {
        convexTrigger = "PIR";
        feedingMethod = "Automatic";
      } else if (triggerSource === "WEB") {
        convexTrigger = "Web";
        feedingMethod = "Manual";
      } else if (triggerSource === "MANUAL_BUTTON") {
        convexTrigger = "Button";
        feedingMethod = "Manual";
      }

      // 1. Add to feed history
      await ctx.runMutation(api.feedHistory.add, {
        ownerId,
        amountDispensed: amountDispensed !== undefined ? parseFloat(amountDispensed) : 0,
        feedingMethod,
        triggerSource: convexTrigger,
        completed: true,
        temperature: temperature !== undefined ? parseFloat(temperature) : 22.4,
        humidity: humidity !== undefined ? parseFloat(humidity) : 45.0,
      });

      // 2. Clear pendingFeedRequest
      await ctx.runMutation(api.deviceSettings.update, {
        ownerId,
        pendingFeedRequest: false,
      });

      return new Response(JSON.stringify({ success: true }), {
        status: 200,
        headers: {
          "Content-Type": "application/json",
          "Access-Control-Allow-Origin": "*",
        },
      });
    } catch (error: any) {
      return new Response(JSON.stringify({ error: error.message }), {
        status: 500,
        headers: { "Content-Type": "application/json" },
      });
    }
  }),
});

export default http;
