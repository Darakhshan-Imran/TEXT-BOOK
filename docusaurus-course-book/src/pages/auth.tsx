import React, { useState } from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/auth/LoginForm';
import SignupForm from '../components/auth/SignupForm';
import { useAuth } from '@site/src/auth/auth-context';
import styles from './AuthPage.module.css';

const AuthPage: React.FC = () => {
  const [isLoginView, setIsLoginView] = useState(true);
  const { user, isLoading } = useAuth();

  return (
    <Layout title="Authentication" description="Login or sign up to access the course content">
      <div className={styles.authPageWrapper}>
        <div className={styles.authCard}>
          {isLoginView ? (
            <LoginForm onSwitchToSignup={() => setIsLoginView(false)} />
          ) : (
            <SignupForm onSwitchToLogin={() => setIsLoginView(true)} />
          )}
        </div>
      </div>
    </Layout>
  );
};

export default AuthPage;
