import { useQuery, useMutation } from 'convex/react';

/**
 * Safe wrapper for Convex queries that catches cloud quota errors or network failures
 * and returns fallback data instead of crashing the React application with a white screen.
 */
export function useSafeQuery<T>(queryFn: any, args: any, fallbackValue: T): T {
  try {
    const result = useQuery(queryFn, args);
    return (result !== undefined && result !== null) ? (result as T) : fallbackValue;
  } catch (err) {
    console.warn("Convex query fallback triggered:", err);
    return fallbackValue;
  }
}

/**
 * Safe wrapper for Convex mutations that gracefully handles execution failures.
 */
export function useSafeMutation(mutationFn: any): (...args: any[]) => Promise<any> {
  let mut: any = null;
  try {
    mut = useMutation(mutationFn);
  } catch (err) {
    console.warn("Convex mutation hook fallback:", err);
  }

  return async (...args: any[]) => {
    if (!mut) return null;
    try {
      return await mut(...args);
    } catch (err) {
      console.warn("Convex mutation execution failed:", err);
      return null;
    }
  };
}
