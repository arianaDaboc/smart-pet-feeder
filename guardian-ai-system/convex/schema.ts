import { defineSchema, defineTable } from "convex/server";
import { v } from "convex/values";

export default defineSchema({
  users: defineTable({
    clerkId: v.string(),
    email: v.string(),
    fullName: v.optional(v.string()),
    image: v.optional(v.string()),
    createdAt: v.float64(),
    lastLoginAt: v.optional(v.float64()),
    updatedAt: v.optional(v.float64()),
    firstName: v.optional(v.string()),
    lastName: v.optional(v.string()),
  }).index("by_clerkId", ["clerkId"]),
  
  pets: defineTable({
    ownerId: v.optional(v.string()), // Clerk user ID
    userId: v.optional(v.string()),
    name: v.string(),
    species: v.string(), // "Cat" | "Dog" | "Human" | "Unknown"
    breed: v.optional(v.string()),
    age: v.optional(v.string()),
    profileImage: v.optional(v.string()), // Convex storage ID or external URL
    trainingImages: v.optional(v.array(v.string())), // Support multiple uploaded images
    aiModelStatus: v.optional(v.string()), // "Trained" | "Training" | "Not Trained"
    isActive: v.optional(v.boolean()), // Soft deletion status
    createdAt: v.optional(v.float64()),
    aiLabel: v.optional(v.string()),
    allowedToFeed: v.optional(v.boolean()),
    dailyGoalGrams: v.optional(v.float64()),
    weightKg: v.optional(v.float64()),
    lifeStage: v.optional(v.string()),
    neutered: v.optional(v.boolean()),
    activityLevel: v.optional(v.string()),
    bodyConditionScore: v.optional(v.float64()),
    targetWeightKg: v.optional(v.float64()),
    foodKcalPer100g: v.optional(v.float64()),
    foodType: v.optional(v.string()),
    allergies: v.optional(v.string()),
    medicalConditions: v.optional(v.string()),
  }).index("by_ownerId", ["ownerId"]),

  deviceSettings: defineTable({
    ownerId: v.string(), // Clerk ID
    foodPortion: v.float64(),
    cooldownMinutes: v.float64(),
    maximumTemperature: v.float64(),
    initialFoodAmount: v.float64(),
    estimatedFoodRemaining: v.float64(),
    foodContainerCapacity: v.float64(), // Capacity of container in grams
    cameraStreamUrl: v.string(),
    wifiSSID: v.string(),
    notificationsEnabled: v.object({
      feedingSuccessful: v.boolean(),
      lowFoodWarning: v.boolean(),
      criticalOverheat: v.boolean(),
    }),
    updatedAt: v.float64(),
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
    activeDietPetId: v.optional(v.string()),
    activeDietPetName: v.optional(v.string()),
    activeDietMealsPerDay: v.optional(v.float64()),
    previousFoodPortion: v.optional(v.float64()),
  }).index("by_ownerId", ["ownerId"]),

  feedHistory: defineTable({
    ownerId: v.string(), // Clerk ID
    timestamp: v.float64(),
    amountDispensed: v.float64(),
    feedingMethod: v.union(v.literal("Manual"), v.literal("Automatic")),
    triggerSource: v.union(v.literal("PIR"), v.literal("Button"), v.literal("Web"), v.literal("Bluetooth")),
    completed: v.boolean(),
    temperature: v.float64(),
    humidity: v.float64(),
  }).index("by_ownerId", ["ownerId"]),

  notifications: defineTable({
    ownerId: v.string(), // Clerk ID
    title: v.string(),
    message: v.string(),
    type: v.union(
      v.literal("Feeding"),
      v.literal("Detection"),
      v.literal("Warning"),
      v.literal("Temperature"),
      v.literal("AI")
    ),
    relatedPetId: v.optional(v.string()),
    read: v.boolean(),
    createdAt: v.float64(),
  }).index("by_ownerId", ["ownerId"]),

  aiRecognition: defineTable({
    ownerId: v.string(),
    timestamp: v.float64(),
    speciesDetected: v.string(),
    confidence: v.float64(),
    authorized: v.boolean(),
    petMatch: v.optional(v.string()), // "AUTHORIZED" | "UNAUTHORIZED" | "UNKNOWN"
    petId: v.optional(v.string()),
    recognizedPetId: v.optional(v.string()),
    recognizedPetName: v.optional(v.string()),
    imageUrl: v.string(),
    notificationSent: v.boolean(),
  }).index("by_ownerId", ["ownerId"]),

  systemLogs: defineTable({
    ownerId: v.optional(v.string()),
    timestamp: v.float64(),
    category: v.string(),
    message: v.string(),
    severity: v.string(),
  }),
});
