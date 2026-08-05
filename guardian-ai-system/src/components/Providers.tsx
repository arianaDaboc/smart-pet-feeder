import React from 'react';
import { ClerkProvider } from "@clerk/clerk-react";
import { ConvexProvider } from "convex/react";
import { convex } from "../lib/convex";
import { CLERK_KEY, isConfigured } from "../lib/env";
import ConfigRequired from "./ConfigRequired";

interface ProvidersProps {
  children: React.ReactNode;
}

class ErrorBoundary extends React.Component<{ children: React.ReactNode }, { hasError: boolean; error: any }> {
  constructor(props: { children: React.ReactNode }) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: any) {
    return { hasError: true, error };
  }

  componentDidCatch(error: any, errorInfo: any) {
    console.warn("Caught backend error:", error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div className="min-h-screen bg-slate-900 text-white flex flex-col items-center justify-center p-6 text-center">
          <div className="w-16 h-16 rounded-full bg-amber-500/20 text-amber-400 flex items-center justify-center mb-4">
            <span className="material-symbols-outlined text-3xl">cloud_off</span>
          </div>
          <h2 className="text-xl font-bold mb-2">Local Mode Active</h2>
          <p className="text-sm text-slate-300 max-w-md mb-6">
            Convex cloud limits exceeded or offline. Guardian AI local recognition engine is active in browser.
          </p>
          <button
            onClick={() => window.location.reload()}
            className="px-6 py-2.5 bg-primary text-white text-xs font-bold rounded-xl shadow-lg hover:bg-primary-container transition-all"
          >
            Retry Connection
          </button>
          <div className="mt-8 text-left w-full max-w-4xl bg-slate-950 p-4 rounded-xl border border-slate-800 text-xs font-mono">
            {this.props.children}
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

export const Providers: React.FC<ProvidersProps> = ({ children }) => {
  if (!isConfigured() || !convex) {
    return <ConfigRequired />;
  }

  return (
    <ClerkProvider publishableKey={CLERK_KEY}>
      <ErrorBoundary>
        <ConvexProvider client={convex}>
          {children}
        </ConvexProvider>
      </ErrorBoundary>
    </ClerkProvider>
  );
};

export default Providers;
