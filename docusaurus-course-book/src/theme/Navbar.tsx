import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useAuth } from '@site/src/auth/auth-context';

const AuthNavbarWrapper = (props) => {
  const { user, signOut } = useAuth();

  // Handle logout
  const handleLogout = async () => {
    await signOut();
    // Redirect to auth page after logout
    if (typeof window !== 'undefined') {
      window.location.href = '/auth';
    }
  };

  return (
    <>
      <Navbar {...props} />
      {/* Add auth-specific UI if needed */}
      {user && (
        <div className="navbar__auth-user">
          {/* <span className="navbar__user-email">{user.email}</span> */}
          {/* <button
            className="button button--sm button--secondary"
            onClick={handleLogout}
          >
            Logout
          </button> */}
        </div>
      )}
    </>
  );
};

export default AuthNavbarWrapper;