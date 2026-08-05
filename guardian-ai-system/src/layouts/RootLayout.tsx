import React, { useEffect, useState } from 'react';
import { Outlet, useLocation } from 'react-router-dom';
import { useUser } from '@clerk/clerk-react';
import { useMutation, useQuery } from 'convex/react';
import { api } from '../../convex/_generated/api';
import { useFeederStore } from '../store/useFeederStore';
import Sidebar from '../components/Sidebar';
import Header from '../components/Header';

export const RootLayout: React.FC = () => {
  const location = useLocation();
  const { user } = useUser();
  const syncUser = useMutation(api.users.sync);
  
  const ownerId = user?.id || '';
  let convexSettings = undefined;
  try {
    convexSettings = useQuery(api.deviceSettings.get, { ownerId });
  } catch (e) {
    console.warn("Convex query warning:", e);
  }
  const { syncFromConvex } = useFeederStore();

  const [sidebarOpen, setSidebarOpen] = useState(false);

  // Synchronize Convex device settings to Zustand store
  useEffect(() => {
    if (convexSettings) {
      syncFromConvex(convexSettings);
    }
  }, [convexSettings, syncFromConvex]);

  // Automatically sync authenticated user profile details to Convex
  useEffect(() => {
    if (user && syncUser) {
      syncUser({
        clerkId: user.id,
        email: user.primaryEmailAddress?.emailAddress || '',
        fullName: user.fullName || '',
        image: user.imageUrl || '',
      }).catch(() => {
        // Silently ignore cloud sync errors
      });
    }
  }, [user, syncUser]);

  // Resolve header title based on current path
  const getHeaderTitle = (pathname: string): string => {
    switch (pathname) {
      case '/':
        return 'Guardian AI';
      case '/camera':
        return 'Live Stream';
      case '/pets':
        return 'My Pets';
      case '/history':
        return 'Activity History';
      case '/insights':
        return 'Health & Insights';
      case '/notifications':
        return 'Notifications';
      case '/settings':
        return 'Guardian AI';
      case '/profile':
        return 'User Profile';
      default:
        return 'Guardian AI';
    }
  };

  const title = getHeaderTitle(location.pathname);

  // Close sidebar on navigation change (mobile)
  useEffect(() => {
    setSidebarOpen(false);
  }, [location.pathname]);

  return (
    <div className="min-h-screen bg-surface flex">
      {/* Sidebar Navigation */}
      <Sidebar isOpen={sidebarOpen} onClose={() => setSidebarOpen(false)} />

      {/* Main Content Area */}
      <div className="flex-1 flex flex-col min-h-screen lg:pl-[260px] w-full">
        {/* Sticky Topbar Header */}
        <Header title={title} onMenuClick={() => setSidebarOpen(true)} />

        {/* Dynamic Page Router Canvas */}
        <main className="flex-1 w-full overflow-x-hidden">
          <Outlet />
        </main>
      </div>
    </div>
  );
};

export default RootLayout;
