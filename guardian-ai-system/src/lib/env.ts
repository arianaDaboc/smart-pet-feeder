export const CLERK_KEY = import.meta.env.VITE_CLERK_PUBLISHABLE_KEY || '';
export const CONVEX_URL = import.meta.env.VITE_CONVEX_URL || '';

export const isConfigured = (): boolean => {
  return CLERK_KEY.trim() !== '' && CONVEX_URL.trim() !== '';
};
