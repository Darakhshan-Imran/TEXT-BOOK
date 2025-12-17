import React from 'react';
import { AuthProvider } from '../auth/auth-context';
import Layout from '@theme/Layout';

export default function AuthLayout({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      <Layout>{children}</Layout>
    </AuthProvider>
  );
}