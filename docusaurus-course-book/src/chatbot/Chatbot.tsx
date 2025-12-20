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
  const [isMinimized, setIsMinimized] = useState(false);
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
    if (isMinimized) {
      // If minimized, restore the panel
      setIsMinimized(false);
    } else {
      setIsOpen(!isOpen);
    }
  };

  const handleClose = () => {
    setIsOpen(false);
    setIsMinimized(false);
  };

  const handleMinimize = () => {
    setIsMinimized(true);
  };

  const userName = user?.name || user?.email?.split('@')[0] || 'Student';

  return (
    <>
      <ChatPanel
        isOpen={isOpen && !isMinimized}
        onClose={handleClose}
        onMinimize={handleMinimize}
        userName={userName}
      />
      <ChatButton onClick={handleToggle} isOpen={isOpen && !isMinimized} isMinimized={isMinimized} />
    </>
  );
};

export default Chatbot;
