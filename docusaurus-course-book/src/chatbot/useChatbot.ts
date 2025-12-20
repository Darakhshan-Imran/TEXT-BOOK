/**
 * React hook for chatbot API interactions.
 * Handles sending messages and receiving AI responses.
 */

import { useState, useCallback } from 'react';
import { getChatbotApiUrl, chatbotConfig } from './config';
import { Message } from './ChatMessage';

interface UseChatbotReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (content: string) => Promise<void>;
  clearMessages: () => void;
  clearError: () => void;
}

interface ChatApiResponse {
  answer: string;
  sources: Array<{
    chapter: string;
    section: string;
    relevance: number;
  }>;
  session_id: string;
}

/**
 * Hook for managing chatbot state and API communication.
 */
export const useChatbot = (): UseChatbotReturn => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);

  /**
   * Send a message to the chatbot API.
   */
  const sendMessage = useCallback(async (content: string) => {
    // Validate message
    if (content.length < chatbotConfig.minMessageLength) {
      setError('Message is too short');
      return;
    }
    if (content.length > chatbotConfig.maxMessageLength) {
      setError('Message is too long');
      return;
    }

    // Get auth token
    const token = localStorage.getItem('access_token');
    if (!token) {
      setError('Please log in to use the chatbot');
      return;
    }

    // Add user message
    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content,
      timestamp: new Date(),
    };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(getChatbotApiUrl(chatbotConfig.endpoints.query), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          question: content,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          setError('Your session has expired. Please log in again.');
          return;
        }
        const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
        throw new Error(errorData.detail || `Error: ${response.status}`);
      }

      const data: ChatApiResponse = await response.json();

      // Update session ID
      if (data.session_id) {
        setSessionId(data.session_id);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: data.answer,
        timestamp: new Date(),
        sources: data.sources,
      };
      setMessages(prev => [...prev, assistantMessage]);

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
      console.error('Chatbot error:', err);
    } finally {
      setIsLoading(false);
    }
  }, [sessionId]);

  /**
   * Clear all messages and start a new conversation.
   */
  const clearMessages = useCallback(() => {
    setMessages([]);
    setSessionId(null);
    setError(null);
  }, []);

  /**
   * Clear the current error.
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    clearMessages,
    clearError,
  };
};

export default useChatbot;
