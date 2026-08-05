import { useEffect, useState } from 'react';

const STORAGE_KEY = 'guardian_profile_image';
const CHANGE_EVENT = 'guardian-profile-image-change';
const NAME_STORAGE_KEY = 'profile_name';
const NAME_CHANGE_EVENT = 'guardian-profile-name-change';

export function useProfileImage() {
  const [profileImage, setProfileImageState] = useState(() => localStorage.getItem(STORAGE_KEY) || '');

  useEffect(() => {
    const syncImage = () => setProfileImageState(localStorage.getItem(STORAGE_KEY) || '');
    window.addEventListener('storage', syncImage);
    window.addEventListener(CHANGE_EVENT, syncImage);
    return () => {
      window.removeEventListener('storage', syncImage);
      window.removeEventListener(CHANGE_EVENT, syncImage);
    };
  }, []);

  const setProfileImage = (value: string) => {
    try {
      if (value) localStorage.setItem(STORAGE_KEY, value);
      else localStorage.removeItem(STORAGE_KEY);
      setProfileImageState(value);
      window.dispatchEvent(new Event(CHANGE_EVENT));
      return true;
    } catch {
      return false;
    }
  };

  return { profileImage, setProfileImage };
}

export function useProfileName() {
  const [profileName, setProfileNameState] = useState(() => localStorage.getItem(NAME_STORAGE_KEY) || 'Guardian User');

  useEffect(() => {
    const syncName = () => setProfileNameState(localStorage.getItem(NAME_STORAGE_KEY) || 'Guardian User');
    window.addEventListener('storage', syncName);
    window.addEventListener(NAME_CHANGE_EVENT, syncName);
    return () => {
      window.removeEventListener('storage', syncName);
      window.removeEventListener(NAME_CHANGE_EVENT, syncName);
    };
  }, []);

  const setProfileName = (value: string) => {
    localStorage.setItem(NAME_STORAGE_KEY, value);
    setProfileNameState(value);
    window.dispatchEvent(new Event(NAME_CHANGE_EVENT));
  };

  return { profileName, setProfileName };
}
