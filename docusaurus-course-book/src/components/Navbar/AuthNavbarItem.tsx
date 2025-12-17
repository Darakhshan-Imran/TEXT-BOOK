import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import NavbarItem from '@theme/NavbarItem';
import type { LinkNavbarItemProps } from '@theme/NavbarItem';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface AuthNavbarItemProps {
  [key: string]: any;
}

const AuthNavbarItem: React.FC<AuthNavbarItemProps> = (props) => {
  const { user, signOut, isLoading } = useAuth();
  const authUrl = useBaseUrl('/auth');
  const homeUrl = useBaseUrl('/');
  const logoutIconUrl = useBaseUrl('/img/logout-icon.svg');
  const [isDropdownOpen, setIsDropdownOpen] = useState<boolean>(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const handleLogout = async (e: React.MouseEvent<HTMLAnchorElement>): Promise<void> => {
    e.preventDefault();
    setIsDropdownOpen(false);
    await signOut();
    // Redirect to home page after logout
    window.location.href = homeUrl;
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent): void => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    };

    if (isDropdownOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isDropdownOpen]);

  // If still loading, don't show anything or show a placeholder
  if (isLoading) {
    return (
      <NavbarItem
        {...props}
        label="..."
        to="#"
        onClick={(e: React.MouseEvent) => e.preventDefault()}
      />
    );
  }

  // Get user display name and initials
  const getDisplayName = (): string => {
    if (user.first_name && user.last_name) {
      return `${user.first_name} ${user.last_name}`;
    }
    if (user.first_name) {
      return user.first_name;
    }
    if (user.username) {
      return user.username;
    }
    return user.email || 'Account';
  };

  const getInitials = (): string => {
    if (user.first_name && user.last_name) {
      return `${user.first_name[0]}${user.last_name[0]}`.toUpperCase();
    }
    if (user.first_name) {
      return user.first_name.substring(0, 2).toUpperCase();
    }
    if (user.username) {
      return user.username.substring(0, 2).toUpperCase();
    }
    if (user.email) {
      return user.email.substring(0, 2).toUpperCase();
    }
    return 'U';
  };

  // If user is authenticated, show user dropdown with avatar
  if (user) {
    const initials = getInitials();

    return (
      <div
        ref={dropdownRef}
        style={{
          position: 'relative',
          display: 'flex',
          alignItems: 'center',
        }}
      >
        <button
          onClick={(e: React.MouseEvent<HTMLButtonElement>) => {
            e.preventDefault();
            e.stopPropagation();
            setIsDropdownOpen(!isDropdownOpen);
          }}
          type="button"
          title={getDisplayName()}
          style={{
            display: 'flex',
            alignItems: 'center',
            border: 'none',
            background: 'none',
            cursor: 'pointer',
            padding: '6px',
            borderRadius: '50%',
            transition: 'transform 0.2s, box-shadow 0.2s',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'scale(1.05)';
            e.currentTarget.style.boxShadow = '0 2px 8px rgba(0,0,0,0.15)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'scale(1)';
            e.currentTarget.style.boxShadow = 'none';
          }}
        >
          <div
            style={{
              width: '36px',
              height: '36px',
              borderRadius: '50%',
              background: 'linear-gradient(135deg, var(--ifm-color-primary), var(--ifm-color-primary-dark))',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: 'white',
              fontWeight: '600',
              fontSize: '15px',
              border: '2px solid var(--ifm-color-primary-lighter)',
            }}
          >
            {initials}
          </div>
        </button>

        {isDropdownOpen && (
          <div
            style={{
              position: 'absolute',
              right: '0',
              top: 'calc(100% + 8px)',
              minWidth: '140px',
              backgroundColor: 'var(--ifm-background-surface-color, #fff)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '6px',
              boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
              padding: '8px 0',
              zIndex: 9999,
            }}
          >
            <button
              onClick={(e: React.MouseEvent<HTMLButtonElement>) => {
                e.preventDefault();
                e.stopPropagation();
                handleLogout(e as any);
              }}
              type="button"
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: '8px',
                width: '100%',
                padding: '10px 16px',
                border: 'none',
                background: 'none',
                color: 'var(--ifm-font-color-base)',
                cursor: 'pointer',
                fontSize: '14px',
                transition: 'background-color 0.2s',
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.backgroundColor = 'var(--ifm-color-emphasis-200)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.backgroundColor = 'transparent';
              }}
            >
              <img
                src={logoutIconUrl}
                alt="Logout"
                style={{
                  width: '16px',
                  height: '16px',
                  opacity: 0.8,
                }}
              />
              <span>Logout</span>
            </button>
          </div>
        )}
      </div>
    );
  }

  // If user is not authenticated, show login link
  return (
    <NavbarItem
      {...props}
      label="Login"
      to={authUrl}
    />
  );
};

export default AuthNavbarItem;