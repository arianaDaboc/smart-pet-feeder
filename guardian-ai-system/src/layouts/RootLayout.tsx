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


  useEffect(() => {
    if (convexSettings) {
      syncFromConvex(convexSettings);
    }
  }, [convexSettings, syncFromConvex]);


  useEffect(() => {
    if (user && syncUser) {
      syncUser({
        clerkId: user.id,
        email: user.primaryEmailAddress?.emailAddress || '',
        fullName: user.fullName || '',
        image: user.imageUrl || '',
      }).catch(() => {

      });
    }
  }, [user, syncUser]);


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


  useEffect(() => {
    setSidebarOpen(false);
  }, [location.pathname]);

  return (
    <div className="min-h-screen bg-surface flex">

      <Sidebar isOpen={sidebarOpen} onClose={() => setSidebarOpen(false)} />


      <div className="flex-1 flex flex-col min-h-screen lg:pl-[260px] w-full">

        <Header title={title} onMenuClick={() => setSidebarOpen(true)} />


        <main className="flex-1 w-full overflow-x-hidden">
          <Outlet />
        </main>
      </div>
    </div>
  );
};

export default RootLayout;
