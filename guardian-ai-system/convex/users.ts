import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

export const sync = mutation({
  args: {
    clerkId: v.string(),
    email: v.string(),
    fullName: v.string(),
    image: v.string(),
  },
  handler: async (ctx, args) => {
    const existing = await ctx.db
      .query("users")
      .withIndex("by_clerkId", (q) => q.eq("clerkId", args.clerkId))
      .first();

    const timestamp = Date.now();

    if (existing) {

      await ctx.db.patch(existing._id, {
        email: args.email,
        fullName: args.fullName,
        image: args.image,
        lastLoginAt: timestamp,
        updatedAt: timestamp,
      });
      return existing._id;
    } else {

      const newId = await ctx.db.insert("users", {
        clerkId: args.clerkId,
        email: args.email,
        fullName: args.fullName,
        image: args.image,
        createdAt: timestamp,
        lastLoginAt: timestamp,
        updatedAt: timestamp,
      });
      return newId;
    }
  },
});

export const current = query({
  args: { clerkId: v.string() },
  handler: async (ctx, args) => {
    return await ctx.db
      .query("users")
      .withIndex("by_clerkId", (q) => q.eq("clerkId", args.clerkId))
      .first();
  },
});
