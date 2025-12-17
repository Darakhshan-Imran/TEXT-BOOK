import React from 'react';
import { AuthProvider } from '@site/src/auth/auth-context';

export default function Root({ children }: { children: React.ReactNode }) {
  return <AuthProvider>{children}</AuthProvider>;
}