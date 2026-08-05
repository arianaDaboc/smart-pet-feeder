import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useSafeQuery, useSafeMutation } from '../lib/useSafeConvex';

interface NotificationDoc {
  _id: string;
  ownerId: string;
  title: string;
  message: string;
  type: "Feeding" | "Detection" | "Warning" | "Temperature" | "AI";
  relatedPetId?: string;
  read: boolean;
  createdAt: number;
}

export const Notifications: React.FC = () => {
  const navigate = useNavigate();
  const { user } = useUser();
  const ownerId = user?.id || '';

  const notificationsList = useSafeQuery(api.notifications.list, { ownerId }, []);
  const markRead = useSafeMutation(api.notifications.markRead);
  const clearAll = useSafeMutation(api.notifications.clearAll);
  const deleteNotification = useSafeMutation(api.notifications.remove);

  const [activeFilter, setActiveFilter] = useState<'all' | 'Feeding' | 'Detection' | 'Warning' | 'Temperature' | 'AI'>('all');

  const handleMarkAllAsRead = async () => {
    if (!ownerId) return;
    try {
      await markRead({ ownerId });
    } catch (err) {
      console.error('Failed to mark all as read:', err);
    }
  };

  const handleNotificationClick = async (id: string) => {
    if (!ownerId) return;
    try {
      await markRead({ ownerId, id: id as any });
    } catch (err) {
      console.error('Failed to mark read:', err);
    }
  };

  const handleClearAll = async () => {
    if (!ownerId) return;
    if (confirm('Are you sure you want to clear all notifications?')) {
      try {
        await clearAll({ ownerId });
      } catch (err) {
        console.error('Failed to clear notifications:', err);
      }
    }
  };

  const handleDeleteSingle = async (e: React.MouseEvent, id: string) => {
    e.stopPropagation();
    try {
      await deleteNotification({ id: id as any });
    } catch (err) {
      console.error('Failed to delete single notification:', err);
    }
  };

  // Sort newest first
  const notifications: NotificationDoc[] = (notificationsList || [])
    .slice()
    .sort((a, b) => b.createdAt - a.createdAt);

  const filteredNotifications = notifications.filter(n => {
    if (activeFilter === 'all') return true;
    return n.type === activeFilter;
  });

  // Split notifications into groups (Today, Yesterday, Older)
  const getDayDiff = (timestamp: number) => {
    const now = new Date();
    const date = new Date(timestamp);
    const diffTime = Math.abs(now.getTime() - date.getTime());
    return Math.floor(diffTime / (1000 * 60 * 60 * 24));
  };

  const todayNotifications = filteredNotifications.filter(n => getDayDiff(n.createdAt) === 0);
  const yesterdayNotifications = filteredNotifications.filter(n => getDayDiff(n.createdAt) === 1);
  const olderNotifications = filteredNotifications.filter(n => getDayDiff(n.createdAt) > 1);

  const unreadCount = notifications.filter(n => !n.read).length;

  const formatTimeAgo = (timestamp: number) => {
    const seconds = Math.floor((Date.now() - timestamp) / 1000);
    if (seconds < 60) return 'Just now';
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes}m ago`;
    const hours = Math.floor(minutes / 60);
    if (hours < 24) return `${hours}h ago`;
    const days = Math.floor(hours / 24);
    if (days === 1) return 'Yesterday';
    return `${days}d ago`;
  };

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1000px] mx-auto w-full min-h-screen">
      {/* Page Header */}
      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">Notifications</h3>
          <p className="text-sm text-on-surface-variant">View alerts and warning history.</p>
        </div>
      </div>

      {/* Filter Bar */}
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center gap-2 flex-wrap">
          {['all', 'Feeding', 'Detection', 'Warning', 'Temperature', 'AI'].map((filter) => (
            <button 
              key={filter}
              onClick={() => setActiveFilter(filter as any)}
              className={`px-5 py-2 text-xs font-bold rounded-full shadow-sm active:scale-95 transition-all ${
                activeFilter === filter 
                  ? 'bg-primary text-on-primary' 
                  : 'bg-white border border-outline-variant hover:bg-surface-container-low text-on-surface-variant'
              }`}
            >
              {filter === 'all' ? 'All' : filter}
            </button>
          ))}
        </div>
        
        <div className="flex items-center gap-4">
          <button 
            onClick={handleMarkAllAsRead}
            className="flex items-center gap-2 text-primary font-bold text-xs hover:underline active:scale-95 transition-all"
          >
            <span className="material-symbols-outlined text-[18px]">done_all</span>
            <span>Mark all as read ({unreadCount} unread)</span>
          </button>
          <button 
            onClick={handleClearAll}
            className="flex items-center gap-2 text-on-surface-variant opacity-75 font-bold text-xs hover:text-error active:scale-95 transition-all"
          >
            <span className="material-symbols-outlined text-[18px]">delete_sweep</span>
            <span>Clear All</span>
          </button>
        </div>
      </div>

      {/* Loading State */}
      {notificationsList === undefined && (
        <div className="bg-white rounded-2xl border border-outline-variant/30 overflow-hidden divide-y divide-outline-variant/20">
          {[1, 2, 3].map(n => (
            <div key={n} className="p-6 flex gap-5 animate-pulse">
              <div className="w-12 h-12 rounded-xl bg-surface-container-high shrink-0"></div>
              <div className="flex-1 space-y-2">
                <div className="h-4 bg-surface-container-high rounded w-1/4"></div>
                <div className="h-3 bg-surface-container-high rounded w-3/4"></div>
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Notifications List */}
      {notificationsList !== undefined && (
        <div className="bg-white rounded-2xl shadow-sm border border-outline-variant/30 overflow-hidden">
          {/* Today Group */}
          {todayNotifications.length > 0 && (
            <>
              <div className="px-6 py-4 bg-[#f8f9ff] border-b border-outline-variant/30">
                <span className="text-xs font-bold text-on-surface-variant uppercase tracking-wider">Today</span>
              </div>
              {todayNotifications.map(n => (
                <NotificationCard key={n._id} notification={n} onClick={handleNotificationClick} onDelete={handleDeleteSingle} formatTime={formatTimeAgo} />
              ))}
            </>
          )}

          {/* Yesterday Group */}
          {yesterdayNotifications.length > 0 && (
            <>
              <div className="px-6 py-4 bg-[#f8f9ff] border-b border-outline-variant/30">
                <span className="text-xs font-bold text-on-surface-variant uppercase tracking-wider">Yesterday</span>
              </div>
              {yesterdayNotifications.map(n => (
                <NotificationCard key={n._id} notification={n} onClick={handleNotificationClick} onDelete={handleDeleteSingle} formatTime={formatTimeAgo} />
              ))}
            </>
          )}

          {/* Older Group */}
          {olderNotifications.length > 0 && (
            <>
              <div className="px-6 py-4 bg-[#f8f9ff] border-b border-outline-variant/30">
                <span className="text-xs font-bold text-on-surface-variant uppercase tracking-wider">Older</span>
              </div>
              {olderNotifications.map(n => (
                <NotificationCard key={n._id} notification={n} onClick={handleNotificationClick} onDelete={handleDeleteSingle} formatTime={formatTimeAgo} />
              ))}
            </>
          )}

          {/* Empty State */}
          {filteredNotifications.length === 0 && (
            <div className="p-12 text-center flex flex-col items-center">
              <span className="material-symbols-outlined text-4xl text-outline-variant mb-3">notifications_off</span>
              <p className="text-sm font-bold text-on-surface">No alerts in this category</p>
              <p className="text-xs text-on-surface-variant mt-1">You are all caught up!</p>
            </div>
          )}
        </div>
      )}

      {/* Settings Preference Shortcut */}
      <div className="bg-[#f8f9ff] rounded-2xl p-6 flex flex-col sm:flex-row items-start sm:items-center justify-between border border-primary/10 gap-4">
        <div className="flex items-center gap-4">
          <div className="p-3 bg-white rounded-full shadow-sm text-primary">
            <span className="material-symbols-outlined">tune</span>
          </div>
          <div>
            <h3 className="font-bold text-sm text-on-surface">Manage Notification Preferences</h3>
            <p className="text-xs text-on-surface-variant">Control which alerts you receive and how often.</p>
          </div>
        </div>
        <button 
          onClick={() => navigate('/settings')}
          className="px-6 py-2.5 bg-white text-primary border border-outline-variant font-bold text-xs rounded-full shadow-sm hover:bg-surface-container-low transition-all active:scale-95 shrink-0"
        >
          Settings
        </button>
      </div>
    </section>
  );
};

// NotificationCard Sub-component
interface NotificationCardProps {
  notification: NotificationDoc;
  onClick: (id: string) => void;
  onDelete: (e: React.MouseEvent, id: string) => void;
  formatTime: (ts: number) => string;
}

const NotificationCard: React.FC<NotificationCardProps> = ({ notification, onClick, onDelete, formatTime }) => {
  const getIcon = (type: string) => {
    switch (type) {
      case 'Feeding':
        return 'task_alt';
      case 'Detection':
        return 'sensors';
      case 'Warning':
      case 'Temperature':
        return 'device_thermostat';
      default:
        return 'info';
    }
  };

  const getIconClass = (type: string) => {
    switch (type) {
      case 'Feeding':
        return 'bg-primary-container/20 text-primary';
      case 'Detection':
        return 'bg-tertiary-container/20 text-tertiary';
      case 'Warning':
      case 'Temperature':
        return 'bg-error-container/20 text-error';
      default:
        return 'bg-outline-variant/20 text-on-surface-variant';
    }
  };

  return (
    <div 
      onClick={() => onClick(notification._id)}
      className={`relative p-6 flex gap-5 border-b border-outline-variant/30 transition-colors cursor-pointer group ${
        !notification.read ? 'bg-primary/[0.03] hover:bg-primary/[0.07]' : 'hover:bg-surface-container-low'
      }`}
    >
      {!notification.read && (
        <div className="absolute left-0 top-0 bottom-0 w-1 bg-primary"></div>
      )}
      <div className={`flex-shrink-0 w-12 h-12 rounded-xl flex items-center justify-center ${getIconClass(notification.type)}`}>
        <span className="material-symbols-outlined text-xl" style={{ fontVariationSettings: "'FILL' 1" }}>
          {getIcon(notification.type)}
        </span>
      </div>
      <div className="flex-1 space-y-1 min-w-0">
        <div className="flex justify-between items-start">
          <h4 className={`text-sm truncate ${!notification.read ? 'font-bold text-on-surface' : 'text-on-surface'}`}>
            {notification.title}
          </h4>
          <div className="flex items-center gap-2">
            <span className="text-[10px] font-semibold text-on-surface-variant shrink-0">{formatTime(notification.createdAt)}</span>
            <button 
              onClick={(e) => onDelete(e, notification._id)}
              className="p-1 rounded text-on-surface-variant hover:text-error hover:bg-surface-container-highest opacity-0 group-hover:opacity-100 transition-opacity"
              title="Delete notification"
            >
              <span className="material-symbols-outlined text-[16px]">delete</span>
            </button>
          </div>
        </div>
        <p className="text-xs text-on-surface-variant leading-relaxed">{notification.message}</p>
      </div>
    </div>
  );
};

export default Notifications;
