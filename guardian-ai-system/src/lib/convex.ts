import { ConvexReactClient } from "convex/react";
import { ConvexHttpClient } from "convex/browser";
import { CONVEX_URL } from "./env";

export const convex = CONVEX_URL ? new ConvexReactClient(CONVEX_URL) : null;
export const convexHttp = CONVEX_URL ? new ConvexHttpClient(CONVEX_URL) : null;
