import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

export const list = query({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    return await ctx.db
      .query("aiRecognition")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .order("desc")
      .collect();
  },
});

export const add = mutation({
  args: {
    ownerId: v.string(),
    speciesDetected: v.string(),
    confidence: v.float64(),
    authorized: v.boolean(),
    petMatch: v.optional(v.string()),
    petId: v.optional(v.string()),
    recognizedPetId: v.optional(v.string()),
    recognizedPetName: v.optional(v.string()),
    imageUrl: v.string(),
    notificationSent: v.boolean(),
  },
  handler: async (ctx, args) => {
    return await ctx.db.insert("aiRecognition", {
      ownerId: args.ownerId,
      timestamp: Date.now(),
      speciesDetected: args.speciesDetected,
      confidence: args.confidence,
      authorized: args.authorized,
      petMatch: args.petMatch ?? (args.authorized ? "AUTHORIZED" : "UNAUTHORIZED"),
      petId: args.petId,
      recognizedPetId: args.recognizedPetId,
      recognizedPetName: args.recognizedPetName,
      imageUrl: args.imageUrl,
      notificationSent: args.notificationSent,
    });
  },
});

export const clear = mutation({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    const list = await ctx.db
      .query("aiRecognition")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .collect();

    for (const item of list) {
      await ctx.db.delete(item._id);
    }
  },
});
