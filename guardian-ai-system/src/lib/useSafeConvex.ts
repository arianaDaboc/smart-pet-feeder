import { useQuery, useMutation } from 'convex/react';
import type { FunctionArgs, FunctionReference, FunctionReturnType } from 'convex/server';



export function useSafeQuery<Query extends FunctionReference<'query'>, Fallback>(
  queryFn: Query,
  args: FunctionArgs<Query>,
  fallbackValue: Fallback,
): FunctionReturnType<Query> | Fallback {
  try {
    const result = useQuery(queryFn, args);
    return (result !== undefined && result !== null) ? result : fallbackValue;
  } catch (err) {
    console.warn("Convex query fallback triggered:", err);
    return fallbackValue;
  }
}



type SafeMutation<Mutation extends FunctionReference<'mutation'>> =
  keyof FunctionArgs<Mutation> extends never
    ? () => Promise<FunctionReturnType<Mutation> | null>
    : (args: FunctionArgs<Mutation>) => Promise<FunctionReturnType<Mutation> | null>;

export function useSafeMutation<Mutation extends FunctionReference<'mutation'>>(
  mutationFn: Mutation,
): SafeMutation<Mutation> {
  let mut: ReturnType<typeof useMutation<Mutation>> | null = null;
  try {
    mut = useMutation(mutationFn);
  } catch (err) {
    console.warn("Convex mutation hook fallback:", err);
  }

  const safeMutation = async (...callArgs: [] | [FunctionArgs<Mutation>]) => {
    if (!mut) return null;
    try {
      return await mut((callArgs[0] ?? {}) as FunctionArgs<Mutation>);
    } catch (err) {
      console.warn("Convex mutation execution failed:", err);
      return null;
    }
  };
  return safeMutation as SafeMutation<Mutation>;
}
