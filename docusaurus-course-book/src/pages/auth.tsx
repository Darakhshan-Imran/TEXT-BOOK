import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/auth/LoginForm';
import SignupForm from '../components/auth/SignupForm';
import { useAuth } from '@site/src/auth/auth-context';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './AuthPage.module.css';

const AuthPage: React.FC = () => {
  const [isLoginView, setIsLoginView] = useState(true);
  const { user, isLoading } = useAuth();
  const docsUrl = useBaseUrl('/docs');

  // Don't auto-redirect on page load - let users see the auth page
  // They can click "Textbook" or navigate manually if already logged in

  return (
    <Layout title="Authentication" description="Login or sign up to access the course content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__body">
                {isLoginView ? (
                  <LoginForm onSwitchToSignup={() => setIsLoginView(false)} />
                ) : (
                  <SignupForm onSwitchToLogin={() => setIsLoginView(true)} />
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default AuthPage;