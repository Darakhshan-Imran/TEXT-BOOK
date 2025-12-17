import React from 'react';
import { useAuth } from '../../auth/auth-context';
import NavbarItem from '@theme/NavbarItem';
// import type { LinkNavbarItemProps } from '@theme/NavbarItem';
// import type { GlobalStyles as LinkGlobalStyles } from '@docusaurus/theme-classic';

// Define the type for our auth navbar item
interface AuthNavbarItemProps {
  position?: 'left' | 'right';
}

const AuthNavbarItem: React.FC<AuthNavbarItemProps> = ({ position }) => {
  const { user, signOut } = useAuth();

  // Handle logout
  const handleLogout = async () => {
    await signOut();
    // Redirect to auth page after logout
    if (typeof window !== 'undefined') {
      window.location.href = '/auth';
    }
  };

  // If user is authenticated, show logout button
  if (user) {
    return (
      <div className="navbar__item">
        <div className="dropdown dropdown--right dropdown--navbar">
          <button className="button button--secondary navbar__link">{user.email || 'Account'}</button>
          <ul className="dropdown__menu">
            <li>
              <a href="#" className="dropdown__link" onClick={(e) => {
                e.preventDefault();
                handleLogout();
              }}>
                Logout
              </a>
            </li>
          </ul>
        </div>
      </div>
    );
  }

  // If user is not authenticated, show login link
  return (
    <NavbarItem
      type="default"
      label="Login"
      to="/auth"
      position={position}
      className="navbar__item navbar__link"
    />
  );
};

export default AuthNavbarItem;