import React from 'react';
import './chatbot.css';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{
    chapter: string;
    section: string;
    relevance: number;
  }>;
}

interface ChatMessageProps {
  message: Message;
}

/**
 * Individual chat message bubble component.
 * Styles differently for user vs assistant messages.
 */
const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.role === 'user';

  return (
    <div className={`chatbot-message ${isUser ? 'chatbot-message-user' : 'chatbot-message-assistant'}`}>
      <div className="chatbot-message-avatar">
        {isUser ? (
          // User icon
          <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
          </svg>
        ) : (
          // Bot icon
          <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1v1a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-1H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1a7 7 0 0 1 7-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 0 1 2-2M7.5 13A2.5 2.5 0 0 0 5 15.5 2.5 2.5 0 0 0 7.5 18a2.5 2.5 0 0 0 2.5-2.5A2.5 2.5 0 0 0 7.5 13m9 0a2.5 2.5 0 0 0-2.5 2.5 2.5 2.5 0 0 0 2.5 2.5 2.5 2.5 0 0 0 2.5-2.5 2.5 2.5 0 0 0-2.5-2.5z" />
          </svg>
        )}
      </div>
      <div className="chatbot-message-content">
        <div className="chatbot-message-text">{message.content}</div>
        {message.sources && message.sources.length > 0 && (
          <div className="chatbot-message-sources">
            <span className="chatbot-sources-label">Sources:</span>
            {message.sources.map((source, index) => (
              <span key={index} className="chatbot-source-tag">
                {source.chapter}
              </span>
            ))}
          </div>
        )}
        <div className="chatbot-message-time">
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      </div>
    </div>
  );
};

export default ChatMessage;
