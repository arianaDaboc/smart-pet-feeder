import React from 'react';
import { useClerk, useUser } from '@clerk/clerk-react';
import { useProfileImage, useProfileName } from '../hooks/useProfileImage';

export const Profile: React.FC = () => {
  const { signOut, openUserProfile } = useClerk();
  const { user } = useUser();

  const { profileName: userName, setProfileName: setUserName } = useProfileName();
  const [userEmail, setUserEmail] = React.useState(() => localStorage.getItem('profile_email') || 'user@guardian.ai');
  const [isEditing, setIsEditing] = React.useState(false);
  const [tempName, setTempName] = React.useState(userName);
  const [tempEmail, setTempEmail] = React.useState(userEmail);

  const { profileImage: userImage, setProfileImage } = useProfileImage();

  const handleImageUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file || !file.type.startsWith('image/')) return;
    const objectUrl = URL.createObjectURL(file);
    const image = new Image();
    image.onload = () => {
      const maxSize = 512;
      const scale = Math.min(1, maxSize / Math.max(image.naturalWidth, image.naturalHeight));
      const canvas = document.createElement('canvas');
      canvas.width = Math.max(1, Math.round(image.naturalWidth * scale));
      canvas.height = Math.max(1, Math.round(image.naturalHeight * scale));
      const context = canvas.getContext('2d');
      context?.drawImage(image, 0, 0, canvas.width, canvas.height);
      const saved = setProfileImage(canvas.toDataURL('image/jpeg', 0.82));
      URL.revokeObjectURL(objectUrl);
      if (!saved) alert('The photo could not be saved in this browser. Try a smaller image.');
    };
    image.onerror = () => {
      URL.revokeObjectURL(objectUrl);
      alert('The selected file is not a valid image.');
    };
    image.src = objectUrl;
    event.target.value = '';
  };

  const handleSaveProfile = (e: React.FormEvent) => {
    e.preventDefault();
    setUserName(tempName);
    setUserEmail(tempEmail);
    localStorage.setItem('profile_email', tempEmail);
    setIsEditing(false);
  };

  const handleLogout = async () => {
    if (confirm('Are you sure you want to log out?')) {
      try {
        await signOut();
      } catch (err) {
        console.error('Logout error:', err);
      }
    }
  };

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[800px] w-full min-h-screen">

      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">User Profile</h3>
          <p className="text-sm text-on-surface-variant">Manage your account details and login sessions.</p>
        </div>
      </div>

      <div className="space-y-6 pb-20 md:pb-0">

        <section className="bg-white rounded-3xl border border-outline-variant/30 overflow-hidden shadow-sm hover:shadow-md transition-all duration-300">
          <div className="relative h-40 bg-surface-container-highest"></div>
          <div className="px-8 pb-10 -mt-20 relative text-center">
            <div className="inline-block relative">
              <div className="w-40 h-40 rounded-3xl border-4 border-white shadow-xl overflow-hidden bg-surface-container mx-auto flex items-center justify-center">
                {userImage ? (
                  <img
                    alt="User Profile"
                    className="w-full h-full object-cover"
                    src={userImage}
                  />
                ) : (
                  <span className="material-symbols-outlined text-[64px] text-on-surface-variant">person</span>
                )}
              </div>
              <label className="absolute -bottom-2 -right-2 w-11 h-11 rounded-full bg-primary text-white border-4 border-white shadow-lg flex items-center justify-center cursor-pointer hover:scale-105 active:scale-95 transition-transform" title="Upload a photo">
                <span className="material-symbols-outlined text-[20px]">photo_camera</span>
                <input type="file" accept="image/*" className="hidden" onChange={handleImageUpload} />
              </label>
            </div>

            {userImage && (
              <button
                type="button"
                onClick={() => setProfileImage('')}
                className="mt-5 text-[11px] font-bold text-error hover:underline"
              >
                Remove photo
              </button>
            )}

            {!isEditing ? (
              <div className={`${userImage ? 'mt-3' : 'mt-6'} space-y-3`}>
                <h3 className="font-bold text-2xl text-on-surface">{userName}</h3>
                <p className="text-sm text-on-surface-variant font-medium">{userEmail}</p>
                <p className="text-[10px] text-outline font-bold uppercase tracking-wider">
                  Member since: {user?.createdAt ? new Date(user.createdAt).toLocaleDateString([], { month: 'long', day: 'numeric', year: 'numeric' }) : 'N/A'}
                </p>
                <button
                  type="button"
                  onClick={() => {
                    setTempName(userName);
                    setTempEmail(userEmail);
                    setIsEditing(true);
                  }}
                  className="mt-4 px-5 py-2.5 bg-primary/10 hover:bg-primary/20 text-primary font-bold text-xs rounded-xl transition-all inline-flex items-center gap-2"
                >
                  <span className="material-symbols-outlined text-sm">edit</span>
                  Edit Profile
                </button>
              </div>
            ) : (
              <form onSubmit={handleSaveProfile} className="mt-6 max-w-sm mx-auto space-y-4 text-left">
                <div>
                  <label className="text-[11px] font-bold text-on-surface-variant uppercase tracking-wider block mb-1">Full Name</label>
                  <input
                    type="text"
                    required
                    value={tempName}
                    onChange={(e) => setTempName(e.target.value)}
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm font-bold focus:outline-none focus:border-primary"
                  />
                </div>
                <div>
                  <label className="text-[11px] font-bold text-on-surface-variant uppercase tracking-wider block mb-1">Email Address</label>
                  <input
                    type="email"
                    required
                    value={tempEmail}
                    onChange={(e) => setTempEmail(e.target.value)}
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm font-bold focus:outline-none focus:border-primary"
                  />
                </div>
                <div className="flex gap-3 pt-2">
                  <button
                    type="submit"
                    className="flex-1 py-2.5 bg-primary hover:bg-primary-container text-white font-bold text-xs rounded-xl transition-all"
                  >
                    Save
                  </button>
                  <button
                    type="button"
                    onClick={() => setIsEditing(false)}
                    className="px-4 py-2.5 border border-outline-variant text-on-surface-variant font-bold text-xs rounded-xl transition-all"
                  >
                    Cancel
                  </button>
                </div>
              </form>
            )}
          </div>
        </section>

        <section className="bg-white rounded-2xl border border-outline-variant/30 p-6 flex items-center justify-between shadow-sm">
          <div className="flex items-center gap-4">
            <div className="w-12 h-12 rounded-xl bg-primary/10 flex items-center justify-center text-primary">
              <span className="material-symbols-outlined">router</span>
            </div>
            <div>
              <p className="font-bold text-sm text-on-surface">Guardian AI</p>
              <p className="text-xs text-on-surface-variant font-medium">Device Status</p>
            </div>
          </div>
          <div className="flex items-center gap-2 px-3 py-1 bg-primary/10 rounded-full">
            <span className="w-1.5 h-1.5 rounded-full bg-primary animate-pulse"></span>
            <span className="text-[10px] font-bold text-primary">Connected</span>
          </div>
        </section>

        <section className="grid grid-cols-1 md:grid-cols-2 gap-stack-gap">

          <button
            type="button"
            onClick={() => openUserProfile()}
            className="group flex items-center justify-between p-6 bg-white rounded-2xl border border-outline-variant/30 hover:border-primary/50 transition-all duration-200 shadow-sm"
          >
            <div className="flex items-center gap-4">
              <div className="w-12 h-12 rounded-xl bg-primary/10 flex items-center justify-center text-primary group-hover:scale-110 transition-transform">
                <span className="material-symbols-outlined">lock_reset</span>
              </div>
              <div className="text-left">
                <p className="font-bold text-sm text-on-surface">Change Password</p>
                <p className="text-xs text-on-surface-variant font-medium">Update your security</p>
              </div>
            </div>
            <span className="material-symbols-outlined text-on-surface-variant group-hover:translate-x-1 transition-transform">chevron_right</span>
          </button>

          <button
            onClick={handleLogout}
            className="group flex items-center justify-between p-6 bg-white rounded-2xl border border-outline-variant/30 hover:bg-error/5 hover:border-error/30 transition-all duration-200 shadow-sm"
          >
            <div className="flex items-center gap-4">
              <div className="w-12 h-12 rounded-xl bg-error-container/20 flex items-center justify-center text-error group-hover:scale-110 transition-transform">
                <span className="material-symbols-outlined">logout</span>
              </div>
              <div className="text-left">
                <p className="font-bold text-sm text-error">Logout</p>
                <p className="text-xs text-on-surface-variant font-medium">Exit current session</p>
              </div>
            </div>
            <span className="material-symbols-outlined text-error opacity-50 group-hover:opacity-100 group-hover:translate-x-1 transition-transform">chevron_right</span>
          </button>
        </section>

      </div>
    </section>
  );
};

export default Profile;
