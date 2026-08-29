import React from 'react';
import { SignIn } from '@clerk/clerk-react';

export const SignInPage: React.FC = () => {
  return (
    <div className="min-h-screen bg-[#f8f9ff] flex items-center justify-center p-6 relative overflow-hidden">

      <div className="absolute top-0 left-0 w-full h-full bg-[radial-gradient(ellipse_at_top_right,_var(--tw-gradient-stops))] from-primary/10 via-transparent to-transparent pointer-events-none"></div>
      <div className="absolute -bottom-48 -left-48 w-96 h-96 bg-primary/5 rounded-full blur-3xl pointer-events-none"></div>

      <div className="w-full max-w-md relative z-10 flex flex-col items-center">
        <div className="mb-8 text-center space-y-2">
          <h1 className="font-bold text-3xl text-primary tracking-tight">Guardian AI</h1>
          <p className="text-sm text-on-surface-variant font-medium">Please sign in to access your pet feeder dashboard</p>
        </div>

        <div className="w-full shadow-2xl rounded-3xl overflow-hidden border border-outline-variant/30 bg-white">
          <SignIn
            signUpUrl="/sign-up"
            afterSignInUrl="/"
            appearance={{
              elements: {
                formButtonPrimary: 'bg-primary hover:bg-primary-container text-white text-xs font-bold py-2.5 rounded-xl transition-all',
                card: 'shadow-none border-none p-6 w-full',
                headerTitle: 'text-on-surface font-bold text-xl',
                headerSubtitle: 'text-on-surface-variant text-xs',
                socialButtonsBlockButton: 'border border-outline-variant hover:bg-surface-container rounded-xl text-xs font-semibold py-2 transition-colors',
                formFieldInput: 'border border-outline-variant focus:border-primary focus:ring-1 focus:ring-primary rounded-xl px-4 py-2 text-sm',
                footerActionLink: 'text-primary hover:underline font-bold text-xs',
              }
            }}
          />
        </div>
      </div>
    </div>
  );
};

export default SignInPage;
