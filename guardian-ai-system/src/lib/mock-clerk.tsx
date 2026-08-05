import React from 'react';

const mockUser = {
  id: "user_test",
  fullName: "Utilizator Guardian",
  primaryEmailAddress: {
    emailAddress: "utilizator@guardian.ro"
  },
  imageUrl: "",
  createdAt: 1672531200000
};

export const ClerkProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return <>{children}</>;
};

export const SignedIn: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return <>{children}</>;
};

export const SignedOut: React.FC<{ children: React.ReactNode }> = () => {
  return null;
};

export const SignIn: React.FC<any> = () => {
  return (
    <div className="p-6 text-center">
      <h2 className="text-lg font-bold">Mock Sign In</h2>
      <p className="text-sm my-2">Bypassed authentication in dev environment</p>
      <a href="/" className="inline-block bg-primary text-white px-4 py-2 rounded-xl mt-4">Go to Dashboard</a>
    </div>
  );
};

export const SignUp: React.FC<any> = () => {
  return (
    <div className="p-6 text-center">
      <h2 className="text-lg font-bold">Mock Sign Up</h2>
      <p className="text-sm my-2">Bypassed authentication in dev environment</p>
      <a href="/" className="inline-block bg-primary text-white px-4 py-2 rounded-xl mt-4">Go to Dashboard</a>
    </div>
  );
};

export const UserButton: React.FC<any> = () => {
  return (
    <div className="w-8 h-8 rounded-full border border-outline-variant bg-surface-container flex items-center justify-center">
      <span className="material-symbols-outlined text-[18px] text-on-surface-variant">person</span>
    </div>
  );
};

export const useUser = () => {
  return {
    isLoaded: true,
    isSignedIn: true,
    user: mockUser
  };
};

export const useClerk = () => {
  return {
    user: mockUser,
    signOut: () => {
      console.log("Mock logout triggered");
      return Promise.resolve();
    }
  };
};

export const useAuth = () => {
  return {
    isLoaded: true,
    isSignedIn: true,
    userId: "user_test",
    getToken: () => Promise.resolve("mock_token")
  };
};
