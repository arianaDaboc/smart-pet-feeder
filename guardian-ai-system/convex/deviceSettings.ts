import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

// Get device settings. If they do not exist, insert defaults.
export const get = query({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    const settings = await ctx.db
      .query("deviceSettings")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .first();

    if (settings) {
      return {
        ...settings,
        cameraStreamUrl: settings.cameraStreamUrl || "http://192.168.100.49:81/stream",
      };
    }

    // Default configuration if none exists
    return {
      ownerId: args.ownerId,
      foodPortion: 45,
      cooldownMinutes: 30,
      maximumTemperature: 42,
      initialFoodAmount: 2000,
      estimatedFoodRemaining: 2000,
      foodContainerCapacity: 2000,
      cameraStreamUrl: "http://192.168.100.49:81/stream",
      wifiSSID: "Home-Network-2G",
      notificationsEnabled: {
        feedingSuccessful: true,
        lowFoodWarning: true,
        criticalOverheat: true,
      },
      updatedAt: Date.now(),
      pendingFeedRequest: undefined,
      currentTemperature: undefined,
      currentHumidity: undefined,
      currentWeight: undefined,
      deviceStatus: undefined,
      online: false,
      lastSeen: undefined,
      firmwareVersion: undefined,
      wifiRSSI: undefined,
      pendingCommand: undefined,
      rawScaleValue: undefined,
      calibrationFactor: undefined,
    };
  },
});

// Update device settings, or insert if not present
export const update = mutation({
  args: {
    ownerId: v.string(),
    foodPortion: v.optional(v.float64()),
    cooldownMinutes: v.optional(v.float64()),
    maximumTemperature: v.optional(v.float64()),
    initialFoodAmount: v.optional(v.float64()),
    estimatedFoodRemaining: v.optional(v.float64()),
    foodContainerCapacity: v.optional(v.float64()),
    cameraStreamUrl: v.optional(v.string()),
    wifiSSID: v.optional(v.string()),
    notificationsEnabled: v.optional(
      v.object({
        feedingSuccessful: v.boolean(),
        lowFoodWarning: v.boolean(),
        criticalOverheat: v.boolean(),
      })
    ),
    pendingFeedRequest: v.optional(v.boolean()),
    currentTemperature: v.optional(v.float64()),
    currentHumidity: v.optional(v.float64()),
    currentWeight: v.optional(v.float64()),
    deviceStatus: v.optional(v.string()),
    online: v.optional(v.boolean()),
    lastSeen: v.optional(v.float64()),
    firmwareVersion: v.optional(v.string()),
    wifiRSSI: v.optional(v.float64()),
    pendingCommand: v.optional(v.string()),
    rawScaleValue: v.optional(v.float64()),
    calibrationFactor: v.optional(v.float64()),
    clearRawScaleValue: v.optional(v.boolean()),
    clearCalibrationFactor: v.optional(v.boolean()),
    activeDietPetId: v.optional(v.string()),
    activeDietPetName: v.optional(v.string()),
    activeDietMealsPerDay: v.optional(v.float64()),
    previousFoodPortion: v.optional(v.float64()),
    clearActiveDiet: v.optional(v.boolean()),
  },
  handler: async (ctx, args) => {
    const existing = await ctx.db
      .query("deviceSettings")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .first();

    const timestamp = Date.now();
    const patchData: any = { updatedAt: timestamp };
    if (args.foodPortion !== undefined) patchData.foodPortion = args.foodPortion;
    if (args.cooldownMinutes !== undefined) patchData.cooldownMinutes = args.cooldownMinutes;
    if (args.maximumTemperature !== undefined) patchData.maximumTemperature = args.maximumTemperature;
    if (args.initialFoodAmount !== undefined) patchData.initialFoodAmount = args.initialFoodAmount;
    if (args.estimatedFoodRemaining !== undefined) patchData.estimatedFoodRemaining = args.estimatedFoodRemaining;
    if (args.foodContainerCapacity !== undefined) patchData.foodContainerCapacity = args.foodContainerCapacity;
    if (args.cameraStreamUrl !== undefined) patchData.cameraStreamUrl = args.cameraStreamUrl;
    if (args.wifiSSID !== undefined) patchData.wifiSSID = args.wifiSSID;
    if (args.notificationsEnabled !== undefined) patchData.notificationsEnabled = args.notificationsEnabled;
    if (args.pendingFeedRequest !== undefined) patchData.pendingFeedRequest = args.pendingFeedRequest;
    if (args.currentTemperature !== undefined) patchData.currentTemperature = args.currentTemperature;
    if (args.currentHumidity !== undefined) patchData.currentHumidity = args.currentHumidity;
    if (args.currentWeight !== undefined) patchData.currentWeight = args.currentWeight;
    if (args.deviceStatus !== undefined) patchData.deviceStatus = args.deviceStatus;
    if (args.online !== undefined) patchData.online = args.online;
    if (args.lastSeen !== undefined) patchData.lastSeen = args.lastSeen;
    if (args.firmwareVersion !== undefined) patchData.firmwareVersion = args.firmwareVersion;
    if (args.wifiRSSI !== undefined) patchData.wifiRSSI = args.wifiRSSI;
    if (args.pendingCommand !== undefined) {
      patchData.pendingCommand = args.pendingCommand;
    }
    if (args.clearActiveDiet === true) {
      patchData.activeDietPetId = undefined;
      patchData.activeDietPetName = undefined;
      patchData.activeDietMealsPerDay = undefined;
      patchData.previousFoodPortion = undefined;
    } else {
      if (args.activeDietPetId !== undefined) patchData.activeDietPetId = args.activeDietPetId;
      if (args.activeDietPetName !== undefined) patchData.activeDietPetName = args.activeDietPetName;
      if (args.activeDietMealsPerDay !== undefined) patchData.activeDietMealsPerDay = args.activeDietMealsPerDay;
      if (args.previousFoodPortion !== undefined) patchData.previousFoodPortion = args.previousFoodPortion;
    }
    
    if (args.clearRawScaleValue === true) {
      patchData.rawScaleValue = undefined;
    } else if (args.rawScaleValue !== undefined) {
      patchData.rawScaleValue = args.rawScaleValue;
    }

    if (args.clearCalibrationFactor === true) {
      patchData.calibrationFactor = undefined;
    } else if (args.calibrationFactor !== undefined) {
      patchData.calibrationFactor = args.calibrationFactor;
    }

    if (existing) {
      await ctx.db.patch(existing._id, patchData);
      return existing._id;
    } else {
      // Insert new settings row
      const defaultSettings = {
        ownerId: args.ownerId,
        foodPortion: args.foodPortion ?? 45,
        cooldownMinutes: args.cooldownMinutes ?? 30,
        maximumTemperature: args.maximumTemperature ?? 42,
        initialFoodAmount: args.initialFoodAmount ?? 2000,
        estimatedFoodRemaining: args.estimatedFoodRemaining ?? 2000,
        foodContainerCapacity: args.foodContainerCapacity ?? 2000,
        cameraStreamUrl: args.cameraStreamUrl ?? "",
        wifiSSID: args.wifiSSID ?? "Home-Network-2G",
        notificationsEnabled: args.notificationsEnabled ?? {
          feedingSuccessful: true,
          lowFoodWarning: true,
          criticalOverheat: true,
        },
        updatedAt: timestamp,
      };
      return await ctx.db.insert("deviceSettings", defaultSettings);
    }
  },
});
