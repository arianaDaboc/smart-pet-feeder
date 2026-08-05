import { mutation } from "./_generated/server";

export const run = mutation({
  args: {},
  handler: async (ctx) => {
    const ownerId = "user_test";

    // 1. Clear existing data for user_test to start fresh
    const existingPets = await ctx.db
      .query("pets")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", ownerId))
      .collect();
    for (const pet of existingPets) {
      await ctx.db.delete(pet._id);
    }

    const existingFeeds = await ctx.db
      .query("feedHistory")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", ownerId))
      .collect();
    for (const feed of existingFeeds) {
      await ctx.db.delete(feed._id);
    }

    const existingDetections = await ctx.db
      .query("aiRecognition")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", ownerId))
      .collect();
    for (const det of existingDetections) {
      await ctx.db.delete(det._id);
    }

    const existingNotifications = await ctx.db
      .query("notifications")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", ownerId))
      .collect();
    for (const notif of existingNotifications) {
      await ctx.db.delete(notif._id);
    }

    // 2. Insert mock pets
    const whiskersId = await ctx.db.insert("pets", {
      ownerId,
      name: "Whiskers",
      species: "Cat",
      breed: "Shorthair Tabby",
      age: "2 years",
      profileImage: "https://images.unsplash.com/photo-1514888286974-6c03e2ca1dba?auto=format&fit=crop&w=300",
      trainingImages: [
        "https://images.unsplash.com/photo-1514888286974-6c03e2ca1dba?auto=format&fit=crop&w=300",
        "https://images.unsplash.com/photo-1573865526739-10659fec78a5?auto=format&fit=crop&w=300"
      ],
      aiModelStatus: "Trained",
      isActive: true,
      createdAt: Date.now() - 30 * 24 * 60 * 60 * 1000,
      dailyGoalGrams: 90,
      weightKg: 4.2
    });

    const rockyId = await ctx.db.insert("pets", {
      ownerId,
      name: "Rocky",
      species: "Dog",
      breed: "Golden Retriever",
      age: "3 years",
      profileImage: "https://images.unsplash.com/photo-1552053831-71594a27632d?auto=format&fit=crop&w=300",
      trainingImages: [
        "https://images.unsplash.com/photo-1552053831-71594a27632d?auto=format&fit=crop&w=300",
        "https://images.unsplash.com/photo-1583511655857-d19b40a7a54e?auto=format&fit=crop&w=300"
      ],
      aiModelStatus: "Trained",
      isActive: true,
      createdAt: Date.now() - 20 * 24 * 60 * 60 * 1000,
      dailyGoalGrams: 350,
      weightKg: 28.5
    });

    // 3. Insert mock deviceSettings (or update if exists)
    const existingSettings = await ctx.db
      .query("deviceSettings")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", ownerId))
      .first();
    const settingsData = {
      ownerId,
      foodPortion: 45,
      cooldownMinutes: 30,
      maximumTemperature: 42,
      initialFoodAmount: 2000,
      estimatedFoodRemaining: 1450,
      foodContainerCapacity: 2000,
      cameraStreamUrl: "https://images.unsplash.com/photo-1514888286974-6c03e2ca1dba?auto=format&fit=crop&w=640",
      wifiSSID: "Home-Network-2G",
      notificationsEnabled: {
        feedingSuccessful: true,
        lowFoodWarning: true,
        criticalOverheat: true
      },
      updatedAt: Date.now()
    };
    if (existingSettings) {
      await ctx.db.patch(existingSettings._id, settingsData);
    } else {
      await ctx.db.insert("deviceSettings", settingsData);
    }

    // 4. Insert mock feedHistory
    const feedTimes = [
      Date.now() - 2 * 60 * 60 * 1000, // 2 hours ago
      Date.now() - 6 * 60 * 60 * 1000, // 6 hours ago
      Date.now() - 24 * 60 * 60 * 1000, // 1 day ago
      Date.now() - 28 * 60 * 60 * 1000, // ~1.2 days ago
      Date.now() - 48 * 60 * 60 * 1000 // 2 days ago
    ];

    await ctx.db.insert("feedHistory", {
      ownerId,
      timestamp: feedTimes[0],
      amountDispensed: 45,
      feedingMethod: "Automatic",
      triggerSource: "PIR",
      completed: true,
      temperature: 22.4,
      humidity: 48.2
    });

    await ctx.db.insert("feedHistory", {
      ownerId,
      timestamp: feedTimes[1],
      amountDispensed: 45,
      feedingMethod: "Manual",
      triggerSource: "Web",
      completed: true,
      temperature: 23.1,
      humidity: 45.9
    });

    await ctx.db.insert("feedHistory", {
      ownerId,
      timestamp: feedTimes[2],
      amountDispensed: 45,
      feedingMethod: "Automatic",
      triggerSource: "PIR",
      completed: true,
      temperature: 21.8,
      humidity: 50.1
    });

    await ctx.db.insert("feedHistory", {
      ownerId,
      timestamp: feedTimes[3],
      amountDispensed: 45,
      feedingMethod: "Manual",
      triggerSource: "Button",
      completed: true,
      temperature: 24.2,
      humidity: 42.0
    });

    await ctx.db.insert("feedHistory", {
      ownerId,
      timestamp: feedTimes[4],
      amountDispensed: 45,
      feedingMethod: "Automatic",
      triggerSource: "PIR",
      completed: false, // Failed feeding try
      temperature: 22.0,
      humidity: 47.5
    });

    // 5. Insert mock aiRecognition detections
    await ctx.db.insert("aiRecognition", {
      ownerId,
      timestamp: Date.now() - 15 * 60 * 1000, // 15 mins ago
      speciesDetected: "Cat",
      confidence: 0.98,
      authorized: true,
      petId: whiskersId,
      recognizedPetId: whiskersId,
      recognizedPetName: "Whiskers",
      imageUrl: "https://images.unsplash.com/photo-1514888286974-6c03e2ca1dba?auto=format&fit=crop&w=300",
      notificationSent: true
    });

    await ctx.db.insert("aiRecognition", {
      ownerId,
      timestamp: Date.now() - 4 * 60 * 60 * 1000, // 4 hours ago
      speciesDetected: "Dog",
      confidence: 0.97,
      authorized: true,
      petId: rockyId,
      recognizedPetId: rockyId,
      recognizedPetName: "Rocky",
      imageUrl: "https://images.unsplash.com/photo-1552053831-71594a27632d?auto=format&fit=crop&w=300",
      notificationSent: true
    });

    await ctx.db.insert("aiRecognition", {
      ownerId,
      timestamp: Date.now() - 36 * 60 * 60 * 1000, // 36 hours ago
      speciesDetected: "Raccoon",
      confidence: 0.89,
      authorized: false,
      imageUrl: "https://images.unsplash.com/photo-1497250681960-ef046c08a56e?auto=format&fit=crop&w=300",
      notificationSent: true
    });

    // 6. Insert mock notifications
    await ctx.db.insert("notifications", {
      ownerId,
      title: "Authorized Pet Fed",
      message: "Whiskers verified and fed 45g of kibble.",
      type: "Feeding",
      relatedPetId: whiskersId,
      read: false,
      createdAt: Date.now() - 15 * 60 * 1000
    });

    await ctx.db.insert("notifications", {
      ownerId,
      title: "Unrecognized Animal Alert",
      message: "An unauthorized Raccoon was detected attempting to access the feeder.",
      type: "Warning",
      read: false,
      createdAt: Date.now() - 36 * 60 * 60 * 1000
    });

    await ctx.db.insert("notifications", {
      ownerId,
      title: "System Overheat Warning",
      message: "Telemetry reported internal temperature of 41.5°C close to max threshold.",
      type: "Temperature",
      read: true,
      createdAt: Date.now() - 2 * 24 * 60 * 60 * 1000
    });

    console.log("Seeding mock database completed successfully!");
  }
});
