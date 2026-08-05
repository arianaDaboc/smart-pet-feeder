import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

// List notifications for the owner
export const list = query({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    return await ctx.db
      .query("notifications")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .order("desc")
      .collect();
  },
});

// Add a notification alert
export const add = mutation({
  args: {
    ownerId: v.string(),
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
  },
  handler: async (ctx, args) => {
    return await ctx.db.insert("notifications", {
      ownerId: args.ownerId,
      title: args.title,
      message: args.message,
      type: args.type,
      relatedPetId: args.relatedPetId,
      read: false,
      createdAt: Date.now(),
    });
  },
});

// Mark a single notification or all notifications as read
export const markRead = mutation({
  args: { 
    ownerId: v.string(),
    id: v.optional(v.id("notifications")) 
  },
  handler: async (ctx, args) => {
    if (args.id) {
      await ctx.db.patch(args.id, { read: true });
    } else {
      // Mark all as read for this owner
      const unread = await ctx.db
        .query("notifications")
        .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
        .collect();

      for (const item of unread) {
        if (!item.read) {
          await ctx.db.patch(item._id, { read: true });
        }
      }
    }
  },
});

// Clear all notifications for the owner
export const clearAll = mutation({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    const list = await ctx.db
      .query("notifications")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .collect();

    for (const item of list) {
      await ctx.db.delete(item._id);
    }
  },
});

// Delete a single notification by ID
export const remove = mutation({
  args: { id: v.id("notifications") },
  handler: async (ctx, args) => {
    await ctx.db.delete(args.id);
  },
});
