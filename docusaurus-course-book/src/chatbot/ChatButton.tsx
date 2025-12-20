import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './chatbot.css';

interface ChatButtonProps {
  onClick: () => void;
  isOpen: boolean;
}

/**
 * Floating chat button component (bottom-right corner).
 * Shows a robot image when closed, X icon when open.
 */
const ChatButton: React.FC<ChatButtonProps> = ({ onClick, isOpen }) => {
  const robotImage = useBaseUrl('/img/robo.png');

  return (
    <button
      className={`chatbot-button ${isOpen ? 'chatbot-button-open' : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      title={isOpen ? 'Close chat' : 'Ask AI Assistant'}
    >
      {isOpen ? (
        // Close icon (X)
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <line x1="18" y1="6" x2="6" y2="18" />
          <line x1="6" y1="6" x2="18" y2="18" />
        </svg>
      ) : (
        // Robot image
        <img
          src={robotImage}
          alt="AI Assistant"
          className="chatbot-button-image"
        />
      )}
    </button>
  );
};

export default ChatButton;
