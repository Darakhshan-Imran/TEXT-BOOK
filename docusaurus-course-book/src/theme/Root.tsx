import React from 'react';
import { AuthProvider } from '../auth/auth-context';
import { Chatbot } from '../chatbot';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}