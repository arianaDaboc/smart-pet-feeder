import React from 'react';
import { CLERK_KEY, CONVEX_URL } from '../lib/env';

export const ConfigRequired: React.FC = () => {
  const missingClerk = !CLERK_KEY || CLERK_KEY.trim() === '';
  const missingConvex = !CONVEX_URL || CONVEX_URL.trim() === '';

  return (
    <div className="min-h-screen bg-[#f8f9ff] flex items-center justify-center p-6 font-sans">
      <div className="w-full max-w-2xl bg-white border border-[#bbcabf]/30 rounded-[24px] shadow-[0_10px_25px_-5px_rgba(0,0,0,0.05)] p-8 md:p-12 relative overflow-hidden">

        <div className="absolute -top-24 -right-24 w-64 h-64 bg-[#006c49]/5 rounded-full blur-3xl"></div>

        <div className="relative z-10 flex flex-col items-center text-center">
          <div className="w-16 h-16 rounded-2xl bg-[#006c49]/10 flex items-center justify-center text-[#006c49] mb-6">
            <span className="material-symbols-outlined text-[36px]" style={{ fontVariationSettings: "'FILL' 1" }}>
              settings_input_component
            </span>
          </div>

          <h1 className="text-3xl font-bold text-[#0b1c30] tracking-tight mb-3">
            Guardian AI Setup Required
          </h1>
          <p className="text-base text-[#3c4a42] max-w-md mb-8">
            Before launching the production interface, please configure the Clerk and Convex credentials.
          </p>

          <div className="w-full space-y-4 mb-8 text-left">
            <div className="p-5 rounded-2xl bg-[#f8f9ff] border border-[#bbcabf]/20">
              <div className="flex items-center justify-between mb-3">
                <span className="font-bold text-[#0b1c30] text-sm tracking-wide uppercase">1. Clerk Authentication SDK</span>
                <span className={`px-2.5 py-0.5 rounded-full text-[10px] font-bold ${missingClerk ? 'bg-[#ffdad6] text-[#ba1a1a]' : 'bg-[#10b981]/15 text-[#006c49]'}`}>
                  {missingClerk ? 'MISSING' : 'CONFIGURED'}
                </span>
              </div>
              <p className="text-xs text-[#3c4a42] leading-relaxed mb-3">
                Used to secure user accounts. Setup an app at <a href="https://clerk.com" target="_blank" rel="noreferrer" className="text-[#0058be] hover:underline">clerk.com</a> and add the Publishable Key variable to your local environment.
              </p>
              <div className="bg-white border border-[#bbcabf]/30 p-2.5 rounded-lg font-mono text-[11px] text-[#0b1c30]">
                VITE_CLERK_PUBLISHABLE_KEY=pk_test_...
              </div>
            </div>

            <div className="p-5 rounded-2xl bg-[#f8f9ff] border border-[#bbcabf]/20">
              <div className="flex items-center justify-between mb-3">
                <span className="font-bold text-[#0b1c30] text-sm tracking-wide uppercase">2. Convex Database backend</span>
                <span className={`px-2.5 py-0.5 rounded-full text-[10px] font-bold ${missingConvex ? 'bg-[#ffdad6] text-[#ba1a1a]' : 'bg-[#10b981]/15 text-[#006c49]'}`}>
                  {missingConvex ? 'MISSING' : 'CONFIGURED'}
                </span>
              </div>
              <p className="text-xs text-[#3c4a42] leading-relaxed mb-3">
                Used to store pet profiles, device metrics, and configuration logs. Setup a Convex project using the CLI and supply the URL.
              </p>
              <div className="bg-white border border-[#bbcabf]/30 p-2.5 rounded-lg font-mono text-[11px] text-[#0b1c30]">
                VITE_CONVEX_URL=https:
              </div>
            </div>
          </div>

          <div className="flex flex-col sm:flex-row gap-3 w-full justify-center">
            <button
              onClick={() => window.location.reload()}
              className="px-8 py-3.5 bg-[#006c49] text-white font-semibold text-sm rounded-xl shadow-lg shadow-[#006c49]/20 hover:scale-[1.02] active:scale-95 transition-all flex items-center justify-center gap-2"
            >
              <span className="material-symbols-outlined text-[18px]">refresh</span>
              Check Configuration Again
            </button>
          </div>

          <p className="text-[11px] text-[#3c4a42]/60 mt-8">
            Create an <b>.env.local</b> file in the root of your project folder to local-load these variables.
          </p>
        </div>
      </div>
    </div>
  );
};
export default ConfigRequired;
