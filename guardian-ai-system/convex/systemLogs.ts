import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

// List system logs
export const list = query({
  args: { ownerId: v.optional(v.string()) },
  handler: async (ctx, args) => {
    const logs = await ctx.db.query("systemLogs").order("desc").collect();
    if (args.ownerId) {
      return logs.filter((l) => l.ownerId === args.ownerId);
    }
    return logs;
  },
});

// Add a system log item
export const add = mutation({
  args: {
    ownerId: v.optional(v.string()),
    category: v.string(),
    message: v.string(),
    severity: v.string(),
  },
  handler: async (ctx, args) => {
    return await ctx.db.insert("systemLogs", {
      ownerId: args.ownerId,
      timestamp: Date.now(),
      category: args.category,
      message: args.message,
      severity: args.severity,
    });
  },
});
