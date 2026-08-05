import React from 'react';
import { useNavigate } from 'react-router-dom';
import { useFeederStore } from '../store/useFeederStore';
import { useProfileImage } from '../hooks/useProfileImage';

interface HeaderProps {
  title: string;
  onMenuClick: () => void;
}

export const Header: React.FC<HeaderProps> = ({ title, onMenuClick }) => {
  const { settings } = useFeederStore();
  const navigate = useNavigate();
  const { profileImage: userImage } = useProfileImage();

  return (
    <header className="sticky top-0 z-40 bg-surface/80 backdrop-blur-md border-b border-outline-variant h-[72px] flex justify-between items-center px-4 md:px-8 w-full">
      <div className="flex items-center gap-3">
        {/* Menu toggle button for mobile/tablet */}
        <button 
          onClick={onMenuClick}
          className="p-2 rounded-full hover:bg-surface-container-highest lg:hidden text-on-surface-variant active:scale-95 transition-all flex items-center justify-center shrink-0"
        >
          <span className="material-symbols-outlined text-[24px]">menu</span>
        </button>
        <h2 className="font-semibold text-lg text-primary tracking-tight truncate">{title}</h2>
      </div>

      <div className="flex items-center gap-6">
        {/* Device Connectivity Status */}
        <div className="hidden sm:flex flex-col items-end">
          <p className="text-[10px] font-bold text-primary uppercase tracking-wider">
            {settings.systemOnline ? 'System Online' : 'System Offline'}
          </p>
          <p className="text-[10px] text-on-surface-variant opacity-70">
            Last synced {settings.lastSyncTime}
          </p>
        </div>

        <div className="w-[1px] h-6 bg-outline-variant/60"></div>

        {/* Notifications Icon */}
        <button
          onClick={() => navigate('/notifications')}
          className="p-2 rounded-full hover:bg-surface-container-highest transition-colors relative active:scale-95"
        >
          <span className="material-symbols-outlined">notifications</span>
          <span className="absolute top-2 right-2 w-2 h-2 bg-error rounded-full border-2 border-surface"></span>
        </button>

        <div
          onClick={() => navigate('/profile')}
          className="w-8 h-8 rounded-full bg-surface-container flex items-center justify-center cursor-pointer overflow-hidden border border-outline-variant hover:border-primary transition-all duration-200"
        >
          {userImage ? (
            <img className="w-full h-full object-cover" src={userImage} alt="User Avatar" />
          ) : (
            <span className="material-symbols-outlined text-[18px] text-on-surface-variant">person</span>
          )}
        </div>
      </div>
    </header>
  );
};

export default Header;
