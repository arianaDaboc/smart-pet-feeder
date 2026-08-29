import React, { useState } from 'react';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useSafeQuery } from '../lib/useSafeConvex';
import { useSearchParams } from 'react-router-dom';

export const History: React.FC = () => {
  const [searchParams] = useSearchParams();
  const eventType = searchParams.get('type');
  const feedingOnly = eventType === 'feeding';
  const { user } = useUser();
  const ownerId = user?.id || '';

  const feedHistory = useSafeQuery(api.feedHistory.list, { ownerId }, []);
  const aiRecognition = useSafeQuery(api.aiRecognition.list, { ownerId }, []);
  const notificationsList = useSafeQuery(api.notifications.list, { ownerId }, []);
  const pets = useSafeQuery(api.pets.list, { ownerId }, []);

  const [filterRange, setFilterRange] = useState<'today' | 'week' | 'month'>('today');
  const [petFilter, setPetFilter] = useState<'all' | string>('all');
  const [showPetDropdown, setShowPetDropdown] = useState(false);

  const events = [
    ...feedHistory.map(f => ({
      id: f._id,
      timestamp: f.timestamp,
      type: 'feeding',
      title: 'Food Dispensed',
      message: `Successfully dispensed ${f.amountDispensed}g of food via ${f.feedingMethod} (Trigger: ${f.triggerSource}).`,
      petName: undefined,
      petImageUrl: undefined,
      details: {
        amount: `${f.amountDispensed}g`,
        temp: `${f.temperature}°C`,
        humidity: `${f.humidity}%`,
        statusLabel: f.completed ? 'Normal' : 'Warning',
        statusIcon: f.completed ? 'check_circle' : 'error'
      }
    })),
    ...aiRecognition.map(ai => ({
      id: ai._id,
      timestamp: ai.timestamp,
      type: 'detection',
      title: ai.authorized ? 'Authorized Pet Detected' : 'Unknown Animal Alert',
      message: ai.authorized
        ? `${ai.recognizedPetName || 'Authorized pet'} identified at feeder.`
        : `Unknown ${ai.speciesDetected || 'animal'} detected at feeder frame.`,
      petName: ai.recognizedPetName || 'Unknown',
      petImageUrl: ai.imageUrl,
      details: {
        amount: undefined,
        temp: undefined,
        humidity: undefined,
        duration: '00:00:15',
        statusLabel: ai.authorized ? 'Authorized' : 'Alert',
        statusIcon: ai.authorized ? 'psychology' : 'warning'
      }
    })),
    ...notificationsList.map(n => ({
      id: n._id,
      timestamp: n.createdAt,
      type: 'alert',
      title: n.title,
      message: n.message,
      petName: undefined,
      petImageUrl: undefined,
      details: {
        amount: undefined,
        temp: undefined,
        humidity: undefined,
        statusLabel: n.type,
        statusIcon: 'notifications'
      }
    }))
  ];

  const now = Date.now();
  const startOfToday = new Date().setHours(0, 0, 0, 0);
  const oneWeekAgo = now - 7 * 24 * 60 * 60 * 1000;
  const oneMonthAgo = now - 30 * 24 * 60 * 60 * 1000;

  const filteredEvents = events.filter((event) => {
    if (feedingOnly && event.type !== 'feeding') return false;

    if (filterRange === 'today' && event.timestamp < startOfToday) return false;
    if (filterRange === 'week' && event.timestamp < oneWeekAgo) return false;
    if (filterRange === 'month' && event.timestamp < oneMonthAgo) return false;

    if (petFilter !== 'all' && event.petName !== petFilter) return false;

    return true;
  }).sort((a, b) => b.timestamp - a.timestamp);

  const todayEvents = filteredEvents.filter(e => e.timestamp >= startOfToday);
  const olderEvents = filteredEvents.filter(e => e.timestamp < startOfToday);

  const exportToCSV = () => {
    const headers = ['ID', 'Timestamp', 'Type', 'Title', 'Message', 'Pet Name', 'Status'];
    const rows = filteredEvents.map((e) => [
      e.id,
      new Date(e.timestamp).toISOString(),
      e.type,
      e.title,
      e.message,
      e.petName || 'N/A',
      e.details.statusLabel || 'N/A'
    ]);

    const csvContent = [headers, ...rows].map(row => row.map(v => `"${v}"`).join(',')).join('\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `guardian_ai_history_${new Date().toISOString().split('T')[0]}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1440px] mx-auto w-full min-h-screen">

      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">{feedingOnly ? 'Feeding History' : 'Activity History'}</h3>
          <p className="text-sm text-on-surface-variant">{feedingOnly ? 'View completed and failed food dispensing events.' : 'View dispensing and motion logs in real-time.'}</p>
        </div>
      </div>

      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center bg-surface-container-low p-1 rounded-xl border border-outline-variant/60">
          <button
            onClick={() => setFilterRange('today')}
            className={`px-6 py-2 rounded-lg font-bold text-xs transition-all ${
              filterRange === 'today' ? 'bg-white shadow-sm text-primary' : 'text-on-surface-variant hover:text-on-surface'
            }`}
          >
            Today
          </button>
          <button
            onClick={() => setFilterRange('week')}
            className={`px-6 py-2 rounded-lg font-bold text-xs transition-all ${
              filterRange === 'week' ? 'bg-white shadow-sm text-primary' : 'text-on-surface-variant hover:text-on-surface'
            }`}
          >
            Week
          </button>
          <button
            onClick={() => setFilterRange('month')}
            className={`px-6 py-2 rounded-lg font-bold text-xs transition-all ${
              filterRange === 'month' ? 'bg-white shadow-sm text-primary' : 'text-on-surface-variant hover:text-on-surface'
            }`}
          >
            Month
          </button>
        </div>

        <div className="flex items-center gap-4">
          <div className="relative">
            <button
              onClick={() => setShowPetDropdown(!showPetDropdown)}
              className="flex items-center gap-2 px-4 py-2 bg-white border border-outline-variant rounded-xl font-bold text-xs hover:border-primary transition-colors"
            >
              <span className="material-symbols-outlined text-[20px] text-primary" style={{ fontVariationSettings: "'FILL' 1" }}>
                pets
              </span>
              <span>{petFilter === 'all' ? 'All Pets' : petFilter}</span>
              <span className="material-symbols-outlined text-[18px]">expand_more</span>
            </button>
            {showPetDropdown && (
              <div className="absolute right-0 top-full mt-2 w-48 bg-white shadow-xl rounded-xl border border-outline-variant p-2 z-[60]">
                <div
                  onClick={() => { setPetFilter('all'); setShowPetDropdown(false); }}
                  className="px-4 py-2 hover:bg-surface-container-low rounded-lg cursor-pointer font-bold text-xs"
                >
                  All Pets
                </div>
                {pets.map(p => (
                  <div
                    key={p._id}
                    onClick={() => { setPetFilter(p.name); setShowPetDropdown(false); }}
                    className="px-4 py-2 hover:bg-surface-container-low rounded-lg cursor-pointer font-bold text-xs"
                  >
                    {p.name} ({p.species})
                  </div>
                ))}
              </div>
            )}
          </div>

          <button
            onClick={exportToCSV}
            className="flex items-center gap-2 px-4 py-2 bg-primary text-on-primary rounded-xl font-bold text-xs shadow-lg shadow-primary/20 hover:scale-[1.02] active:scale-95 transition-all"
          >
            <span className="material-symbols-outlined text-[20px]">download</span>
            <span>Export CSV</span>
          </button>
        </div>
      </div>

      <div className="space-y-12">

        {todayEvents.length > 0 && (
          <div>
            <h3 className="font-bold text-xs text-on-surface-variant uppercase tracking-[0.2em] mb-6 flex items-center gap-4">
              Today
              <span className="h-[1px] flex-1 bg-outline-variant/60"></span>
            </h3>
            <div className="space-y-4">
              {todayEvents.map((event) => (
                <TimelineCard key={event.id} event={event} />
              ))}
            </div>
          </div>
        )}

        {olderEvents.length > 0 && (
          <div>
            <h3 className="font-bold text-xs text-on-surface-variant uppercase tracking-[0.2em] mb-6 flex items-center gap-4">
              Older History
              <span className="h-[1px] flex-1 bg-outline-variant/60"></span>
            </h3>
            <div className="space-y-4">
              {olderEvents.map((event) => (
                <TimelineCard key={event.id} event={event} />
              ))}
            </div>
          </div>
        )}
      </div>

      {filteredEvents.length === 0 && (
        <div className="mt-12 flex flex-col md:flex-row items-center gap-8 bg-surface-container-low/50 rounded-3xl p-8 border border-dashed border-outline-variant justify-center text-center">
          <div className="space-y-2">
            <h4 className="font-bold text-lg text-on-surface">No events found</h4>
            <p className="text-xs text-on-surface-variant max-w-md leading-relaxed">
              No historical data matching the current filter filters found in your database.
            </p>
          </div>
        </div>
      )}
    </section>
  );
};

interface TimelineCardProps {
  event: {
    id: string;
    timestamp: number;
    type: string;
    title: string;
    message: string;
    petImageUrl?: string;
    details: {
      amount?: string;
      temp?: string;
      humidity?: string;
      duration?: string;
      statusLabel?: string;
      statusIcon?: string;
    };
  };
}

const TimelineCard: React.FC<TimelineCardProps> = ({ event }) => {
  return (
    <div className="timeline-item group relative pl-12 timeline-line">

      <div className={`absolute left-2 top-2 w-5 h-5 rounded-full border-4 border-white shadow-sm z-10 ${
        event.type === 'feeding'
          ? 'bg-primary'
          : event.type === 'alert'
            ? 'bg-error'
            : 'bg-secondary'
      }`}></div>

      <div className="glass-card p-6 rounded-2xl bg-white flex flex-col lg:flex-row items-start lg:items-center justify-between gap-6 hover:shadow-md transition-all duration-300">
        <div className="flex items-center gap-4 shrink-0">
          {event.petImageUrl ? (
            <div className="w-12 h-12 rounded-full overflow-hidden bg-surface-container-high shrink-0">
              <img className="w-full h-full object-cover" src={event.petImageUrl} alt="Pet Profile" />
            </div>
          ) : (
            <div className={`w-12 h-12 rounded-full flex items-center justify-center shrink-0 ${
              event.type === 'feeding'
                ? 'bg-primary/10 text-primary'
                : event.type === 'alert'
                  ? 'bg-error/10 text-error'
                  : 'bg-secondary/10 text-secondary'
            }`}>
              <span className="material-symbols-outlined text-lg">
                {event.details.statusIcon || 'restaurant'}
              </span>
            </div>
          )}
          <div>
            <div className="flex items-center gap-2">
              <span className="font-bold text-base text-on-surface">{event.title}</span>
              <span className={`px-2 py-0.5 rounded-full font-bold text-[9px] uppercase ${
                event.type === 'feeding'
                  ? 'bg-primary/10 text-primary'
                  : event.type === 'alert'
                    ? 'bg-error/10 text-error'
                    : 'bg-secondary/10 text-secondary'
              }`}>
                {event.type}
              </span>
            </div>
            <p className="text-[11px] text-on-surface-variant font-medium mt-0.5">{event.message}</p>
          </div>
        </div>

        <div className="grid grid-cols-2 sm:grid-cols-4 gap-6 md:gap-12 flex-grow w-full lg:w-auto">
          {event.type === 'feeding' ? (
            <>
              <div className="flex flex-col">
                <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Amount</span>
                <span className="font-bold text-lg text-primary">{event.details.amount}</span>
              </div>
              <div className="flex flex-col">
                <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Temp</span>
                <span className="font-bold text-lg text-on-surface">{event.details.temp}</span>
              </div>
              <div className="flex flex-col">
                <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Humidity</span>
                <span className="font-bold text-lg text-on-surface">{event.details.humidity}</span>
              </div>
              <div className="flex flex-col">
                <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Status</span>
                <span className={`flex items-center gap-1 font-bold text-xs ${
                  event.details.statusLabel === 'Normal' ? 'text-primary' : 'text-tertiary'
                }`}>
                  <span className="material-symbols-outlined text-[16px]">{event.details.statusIcon}</span>
                  {event.details.statusLabel}
                </span>
              </div>
            </>
          ) : (
            <>
              <div className="flex flex-col col-span-2">
                <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Details</span>
                <span className="text-xs font-semibold text-on-surface truncate">{event.message}</span>
              </div>
              {event.details.duration && (
                <div className="flex flex-col">
                  <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Duration</span>
                  <span className="font-bold text-lg text-on-surface">{event.details.duration}</span>
                </div>
              )}
              {event.details.statusLabel && (
                <div className="flex flex-col">
                  <span className="text-[9px] font-bold text-on-surface-variant uppercase mb-1">Status</span>
                  <span className="font-bold text-lg text-on-surface">{event.details.statusLabel}</span>
                </div>
              )}
            </>
          )}
        </div>
      </div>
    </div>
  );
};

export default History;
