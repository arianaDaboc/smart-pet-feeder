import React from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { SignedIn, SignedOut } from '@clerk/clerk-react';
import Providers from './components/Providers';
import RootLayout from './layouts/RootLayout';
import Dashboard from './pages/Dashboard';
import LiveCamera from './pages/LiveCamera';
import RegisteredPets from './pages/RegisteredPets';
import History from './pages/History';
import HealthInsights from './pages/HealthInsights';
import Notifications from './pages/Notifications';
import Settings from './pages/Settings';
import Profile from './pages/Profile';
import SignInPage from './pages/SignInPage';
import SignUpPage from './pages/SignUpPage';

export const App: React.FC = () => {
  return (
    <Providers>
      <BrowserRouter>
        <Routes>

          <Route
            path="/sign-in"
            element={
              <>
                <SignedOut>
                  <SignInPage />
                </SignedOut>
                <SignedIn>
                  <Navigate to="/" replace />
                </SignedIn>
              </>
            }
          />
          <Route
            path="/sign-up"
            element={
              <>
                <SignedOut>
                  <SignUpPage />
                </SignedOut>
                <SignedIn>
                  <Navigate to="/" replace />
                </SignedIn>
              </>
            }
          />

          <Route
            path="*"
            element={
              <>
                <SignedIn>
                  <Routes>
                    <Route element={<RootLayout />}>
                      <Route path="/" element={<Dashboard />} />
                      <Route path="/camera" element={<LiveCamera />} />
                      <Route path="/pets" element={<RegisteredPets />} />
                      <Route path="/history" element={<History />} />
                      <Route path="/insights" element={<HealthInsights />} />
                      <Route path="/notifications" element={<Notifications />} />
                      <Route path="/settings" element={<Settings />} />
                      <Route path="/profile" element={<Profile />} />
                      <Route path="*" element={<Navigate to="/" replace />} />
                    </Route>
                  </Routes>
                </SignedIn>
                <SignedOut>
                  <Navigate to="/sign-in" replace />
                </SignedOut>
              </>
            }
          />
        </Routes>
      </BrowserRouter>
    </Providers>
  );
};

export default App;
