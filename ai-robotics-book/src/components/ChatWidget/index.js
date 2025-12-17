import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatPanel from './ChatPanel';
import useChat from '../../hooks/useChat';
import styles from './styles.module.css';

/**
 * Main ChatWidget component with floating button and panel.
 */
export default function ChatWidget({ pendingSelection, onSelectionHandled }) {
  const [isOpen, setIsOpen] = useState(false);
  const location = useLocation();
  const {
    messages,
    isLoading,
    error,
    sendMessage,
    sendSelectionQuery,
    retryLastMessage,
    clearChat
  } = useChat();

  // Detect if we're on the homepage (navy background) or docs pages (white background)
  const isHomePage = location.pathname === '/' || location.pathname === '/ai-robotics-book/' || location.pathname === '/ai-robotics-book';
  const themeClass = isHomePage ? styles.lightTheme : styles.darkTheme;

  // Handle pending selection from text selection
  useEffect(() => {
    if (pendingSelection && !isLoading) {
      // Open the chat panel
      setIsOpen(true);
      // Send the selection query
      sendSelectionQuery(pendingSelection);
      // Clear the pending selection
      if (onSelectionHandled) {
        onSelectionHandled();
      }
    }
  }, [pendingSelection, isLoading, sendSelectionQuery, onSelectionHandled]);

  const handleToggle = () => {
    setIsOpen(!isOpen);
  };

  const handleClose = () => {
    setIsOpen(false);
  };

  return (
    <>
      {/* Backdrop overlay when chat is open */}
      {isOpen && (
        <div
          className={styles.backdrop}
          onClick={handleClose}
          aria-hidden="true"
        />
      )}

      <div className={`${styles.chatWidgetContainer} ${themeClass}`}>
        {isOpen && (
          <ChatPanel
            messages={messages}
            isLoading={isLoading}
            error={error}
            onSend={sendMessage}
            onClose={handleClose}
            onRetry={error ? retryLastMessage : null}
            onClear={clearChat}
            themeClass={themeClass}
          />
        )}

        <button
          className={styles.chatButton}
          onClick={handleToggle}
          aria-label={isOpen ? 'Close chat' : 'Open chat'}
        >
          {isOpen ? (
            <svg className={styles.chatButtonIcon} viewBox="0 0 24 24">
              <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
            </svg>
          ) : (
            <svg className={styles.chatButtonIcon} viewBox="0 0 24 24">
              <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
            </svg>
          )}
        </button>
      </div>
    </>
  );
}
