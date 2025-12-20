import React, { useState } from 'react';
import { useAuth } from '../auth/auth-context';
import ChatButton from './ChatButton';
import ChatPanel from './ChatPanel';
import './chatbot.css';

/**
 * Main Chatbot component that integrates button and panel.
 * Only visible to logged-in users.
 */
const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const { user, isLoading } = useAuth();

  // Don't render anything while checking auth status
  if (isLoading) {
    return null;
  }

  // Only show chatbot for logged-in users
  if (!user) {
    return null;
  }

  const handleToggle = () => {
    setIsOpen(!isOpen);
  };

  const handleClose = () => {
    setIsOpen(false);
  };

  return (
    <>
      <ChatPanel isOpen={isOpen} onClose={handleClose} />
      <ChatButton onClick={handleToggle} isOpen={isOpen} />
    </>
  );
};

export default Chatbot;
