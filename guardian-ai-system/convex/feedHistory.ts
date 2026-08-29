import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

export const list = query({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    return await ctx.db
      .query("feedHistory")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .order("desc")
      .collect();
  },
});

export const add = mutation({
  args: {
    ownerId: v.string(),
    amountDispensed: v.float64(),
    feedingMethod: v.union(v.literal("Manual"), v.literal("Automatic")),
    triggerSource: v.union(v.literal("PIR"), v.literal("Button"), v.literal("Web"), v.literal("Bluetooth")),
    completed: v.boolean(),
    temperature: v.float64(),
    humidity: v.float64(),
  },
  handler: async (ctx, args) => {
    const timestamp = Date.now();
    const insertedId = await ctx.db.insert("feedHistory", {
      ownerId: args.ownerId,
      timestamp,
      amountDispensed: args.amountDispensed,
      feedingMethod: args.feedingMethod,
      triggerSource: args.triggerSource,
      completed: args.completed,
      temperature: args.temperature,
      humidity: args.humidity,
    });

    const settings = await ctx.db
      .query("deviceSettings")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .first();

    if (settings) {
      const newRemaining = Math.max(0, (settings.estimatedFoodRemaining ?? settings.initialFoodAmount ?? 2000) - args.amountDispensed);
      await ctx.db.patch(settings._id, {
        estimatedFoodRemaining: newRemaining,
        updatedAt: timestamp,
      });
    }

    await ctx.db.insert("notifications", {
      ownerId: args.ownerId,
      title: args.completed ? "Feeding Successful" : "Feeding Failed",
      message: args.completed
        ? `Successfully dispensed ${args.amountDispensed}g of food via ${args.feedingMethod} (Source: ${args.triggerSource}).`
        : `Feeder failed to dispense ${args.amountDispensed}g via ${args.feedingMethod}.`,
      type: "Feeding",
      read: false,
      createdAt: timestamp,
    });

    return insertedId;
  },
});

export const clear = mutation({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    const history = await ctx.db
      .query("feedHistory")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .collect();

    for (const item of history) {
      await ctx.db.delete(item._id);
    }
  },
});
