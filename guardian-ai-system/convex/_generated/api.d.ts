/* eslint-disable */
/**
 * Generated `api` utility.
 *
 * THIS CODE IS AUTOMATICALLY GENERATED.
 *
 * To regenerate, run `npx convex dev`.
 * @module
 */

import type * as aiRecognition from "../aiRecognition.js";
import type * as deviceSettings from "../deviceSettings.js";
import type * as feedHistory from "../feedHistory.js";
import type * as http from "../http.js";
import type * as notifications from "../notifications.js";
import type * as pets from "../pets.js";
import type * as seed from "../seed.js";
import type * as storage from "../storage.js";
import type * as systemLogs from "../systemLogs.js";
import type * as users from "../users.js";

import type {
  ApiFromModules,
  FilterApi,
  FunctionReference,
} from "convex/server";

declare const fullApi: ApiFromModules<{
  aiRecognition: typeof aiRecognition;
  deviceSettings: typeof deviceSettings;
  feedHistory: typeof feedHistory;
  http: typeof http;
  notifications: typeof notifications;
  pets: typeof pets;
  seed: typeof seed;
  storage: typeof storage;
  systemLogs: typeof systemLogs;
  users: typeof users;
}>;

/**
 * A utility for referencing Convex functions in your app's public API.
 *
 * Usage:
 * ```js
 * const myFunctionReference = api.myModule.myFunction;
 * ```
 */
export declare const api: FilterApi<
  typeof fullApi,
  FunctionReference<any, "public">
>;

/**
 * A utility for referencing Convex functions in your app's internal API.
 *
 * Usage:
 * ```js
 * const myFunctionReference = internal.myModule.myFunction;
 * ```
 */
export declare const internal: FilterApi<
  typeof fullApi,
  FunctionReference<any, "internal">
>;

export declare const components: {};
