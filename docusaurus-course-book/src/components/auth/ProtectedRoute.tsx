import React, { useEffect } from 'react';
import { useAuth } from '@site/src/auth/auth-context';

interface ProtectedRouteProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  fallback = (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="text--center">
            <h1>Redirecting to authentication...</h1>
            <p>Please wait while we check your authentication status.</p>
          </div>
        </div>
      </div>
    </div>
  )
}) => {
  const { user, isLoading } = useAuth();

  // Redirect to auth if user is not authenticated
  useEffect(() => {
    if (!isLoading && !user) {
      if (typeof window !== 'undefined') {
        window.location.href = '/auth';
      }
    }
  }, [user, isLoading]);

  if (!isLoading && !user) {
    return fallback;
  }

  // If loading, show nothing or a loading indicator
  if (isLoading) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="text--center">
              <h1>Loading...</h1>
              <p>Please wait while we check your authentication status.</p>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // If user is authenticated, show the children
  return <>{children}</>;
};

export default ProtectedRoute;