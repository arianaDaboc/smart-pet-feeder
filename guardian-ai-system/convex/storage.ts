import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

// Generate upload URL for file storage uploads (images, training sets, snapshots)
export const generateUploadUrl = mutation({
  args: {},
  handler: async (ctx) => {
    return await ctx.storage.generateUploadUrl();
  },
});

// Resolve public URL from storage ID
export const getUrl = query({
  args: { storageId: v.string() },
  handler: async (ctx, args) => {
    return await ctx.storage.getUrl(args.storageId);
  },
});
