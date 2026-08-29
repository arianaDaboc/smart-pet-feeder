import React from 'react';
import { NavLink } from 'react-router-dom';
import { useProfileImage, useProfileName } from '../hooks/useProfileImage';

interface SidebarProps {
  isOpen: boolean;
  onClose: () => void;
}

export const Sidebar: React.FC<SidebarProps> = ({ isOpen, onClose }) => {
  const { profileName: userName } = useProfileName();
  const { profileImage: userImage } = useProfileImage();

  const menuItems = [
    { name: 'Dashboard', path: '/', icon: 'dashboard' },
    { name: 'Live Camera', path: '/camera', icon: 'videocam' },
    { name: 'Registered Pets', path: '/pets', icon: 'pets' },
    { name: 'History', path: '/history', icon: 'history' },
    { name: 'Health & Insights', path: '/insights', icon: 'insights' },
    { name: 'Notifications', path: '/notifications', icon: 'notifications' },
    { name: 'Settings', path: '/settings', icon: 'settings' },
    { name: 'Profile', path: '/profile', icon: 'person' },
  ];

  return (
    <>

      {isOpen && (
        <div
          onClick={onClose}
          className="fixed inset-0 bg-black/40 z-40 lg:hidden backdrop-blur-xs transition-opacity duration-300"
        />
      )}

      <aside className={`fixed left-0 top-0 h-full w-[260px] bg-surface shadow-sm border-r border-outline-variant flex flex-col z-50 transition-transform duration-300 ease-in-out lg:translate-x-0 ${
        isOpen ? 'translate-x-0' : '-translate-x-full'
      }`}>
        <div className="p-6 flex items-center justify-between">
          <div>
            <h1 className="font-bold text-2xl text-primary tracking-tight">Guardian AI</h1>
            <p className="text-on-surface-variant text-xs opacity-75 font-medium tracking-wide">Intelligent Pet Care</p>
          </div>

          <button
            onClick={onClose}
            className="p-1 rounded-full hover:bg-surface-container lg:hidden text-on-surface-variant"
          >
            <span className="material-symbols-outlined">close</span>
          </button>
        </div>

        <nav className="flex-1 px-4 space-y-1 overflow-y-auto scrollbar-hide">
          {menuItems.map((item) => (
            <NavLink
              key={item.name}
              to={item.path}
              onClick={onClose}
              className={({ isActive }) =>
                `flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 group relative ${
                  isActive
                    ? 'text-primary font-bold bg-surface-container-low after:content-[""] after:absolute after:left-0 after:w-1 after:h-6 after:bg-primary after:rounded-full'
                    : 'text-on-surface-variant hover:bg-surface-container-low hover:text-on-surface'
                }`
              }
            >
              <span className="material-symbols-outlined text-[20px]">{item.icon}</span>
              <span className="text-sm font-medium">{item.name}</span>
            </NavLink>
          ))}
        </nav>

        <div className="p-4 border-t border-outline-variant">
          <NavLink
            to="/profile"
            onClick={onClose}
            className="flex items-center gap-3 bg-surface-container-lowest p-3 rounded-2xl shadow-sm border border-outline-variant/30 hover:bg-surface-container-low transition-colors duration-200"
          >
            {userImage ? (
              <img
                className="w-10 h-10 rounded-full object-cover border border-outline-variant/50"
                src={userImage}
                alt={userName}
              />
            ) : (
              <div className="w-10 h-10 rounded-full bg-surface-container flex items-center justify-center border border-outline-variant/50 shrink-0">
                <span className="material-symbols-outlined text-[20px] text-on-surface-variant">person</span>
              </div>
            )}
            <div className="overflow-hidden">
              <p className="text-sm font-bold text-on-surface truncate">{userName}</p>
              <p className="text-[10px] text-on-surface-variant uppercase tracking-wider font-semibold opacity-70">
                Device Owner
              </p>
            </div>
          </NavLink>
        </div>
      </aside>
    </>
  );
};

export default Sidebar;
