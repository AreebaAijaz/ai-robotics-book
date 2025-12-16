import { useState, useCallback, useEffect } from 'react';
import { sendMessage as apiSendMessage, sendSelectionMessage } from '../services/apiClient';

// Generate a unique session ID
const generateSessionId = () => {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
};

// Storage keys
const STORAGE_KEY = 'rag-chatbot-messages';
const SESSION_KEY = 'rag-chatbot-session';

/**
 * Custom hook for managing chat state.
 */
export default function useChat() {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);
  const [lastMessage, setLastMessage] = useState(null);

  // Initialize session ID and load messages from sessionStorage
  useEffect(() => {
    // Get or create session ID
    let storedSessionId = sessionStorage.getItem(SESSION_KEY);
    if (!storedSessionId) {
      storedSessionId = generateSessionId();
      sessionStorage.setItem(SESSION_KEY, storedSessionId);
    }
    setSessionId(storedSessionId);

    // Load messages from sessionStorage
    const storedMessages = sessionStorage.getItem(STORAGE_KEY);
    if (storedMessages) {
      try {
        const parsed = JSON.parse(storedMessages);
        if (Array.isArray(parsed)) {
          setMessages(parsed);
        }
      } catch (e) {
        console.error('Failed to parse stored messages:', e);
      }
    }
  }, []);

  // Save messages to sessionStorage whenever they change
  useEffect(() => {
    if (messages.length > 0) {
      sessionStorage.setItem(STORAGE_KEY, JSON.stringify(messages));
    }
  }, [messages]);

  /**
   * Send a message to the chatbot.
   */
  const sendMessage = useCallback(async (content) => {
    if (!sessionId || !content.trim()) return;

    setError(null);
    setLastMessage(content);

    // Add user message
    const userMessage = { role: 'user', content };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await apiSendMessage(sessionId, content);

      // Add assistant message
      const assistantMessage = {
        role: 'assistant',
        content: response.content,
        citations: response.citations,
        messageId: response.message_id,
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Chat error:', err);
      setError(err.message || 'Failed to send message. Please try again.');
    } finally {
      setIsLoading(false);
    }
  }, [sessionId]);

  /**
   * Send a selection-based message.
   */
  const sendSelectionQuery = useCallback(async (selectedText, question = null) => {
    if (!sessionId || !selectedText.trim()) return;

    setError(null);

    // Build the display message
    const displayContent = question
      ? `[Selected: "${selectedText.slice(0, 50)}${selectedText.length > 50 ? '...' : ''}"]\n\n${question}`
      : `Explain: "${selectedText.slice(0, 100)}${selectedText.length > 100 ? '...' : ''}"`;

    setLastMessage({ type: 'selection', selectedText, question });

    // Add user message
    const userMessage = { role: 'user', content: displayContent };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await sendSelectionMessage(sessionId, selectedText, question);

      // Add assistant message
      const assistantMessage = {
        role: 'assistant',
        content: response.content,
        citations: response.citations,
        messageId: response.message_id,
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Chat error:', err);
      setError(err.message || 'Failed to send message. Please try again.');
    } finally {
      setIsLoading(false);
    }
  }, [sessionId]);

  /**
   * Retry the last failed message.
   */
  const retryLastMessage = useCallback(() => {
    if (!lastMessage) return;

    // Remove the last user message (which failed)
    setMessages((prev) => prev.slice(0, -1));

    if (typeof lastMessage === 'string') {
      sendMessage(lastMessage);
    } else if (lastMessage.type === 'selection') {
      sendSelectionQuery(lastMessage.selectedText, lastMessage.question);
    }
  }, [lastMessage, sendMessage, sendSelectionQuery]);

  /**
   * Clear all messages.
   */
  const clearChat = useCallback(() => {
    setMessages([]);
    setError(null);
    sessionStorage.removeItem(STORAGE_KEY);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    sendSelectionQuery,
    retryLastMessage,
    clearChat,
    sessionId,
  };
}
