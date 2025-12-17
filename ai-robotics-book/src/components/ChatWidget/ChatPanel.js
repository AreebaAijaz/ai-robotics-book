import React, { useRef, useEffect } from 'react';
import Message from './Message';
import ChatInput from './ChatInput';
import styles from './styles.module.css';

/**
 * Chat panel with message list and input.
 */
export default function ChatPanel({
  messages,
  isLoading,
  error,
  onSend,
  onClose,
  onRetry,
  onClear,
  themeClass
}) {
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading]);

  return (
    <div className={`${styles.chatPanel} ${themeClass || ''}`}>
      {/* Header */}
      <div className={styles.panelHeader}>
        <h3 className={styles.panelTitle}>Book Assistant</h3>
        <div className={styles.headerButtons}>
          {messages.length > 0 && onClear && (
            <button
              className={styles.clearButton}
              onClick={onClear}
              aria-label="Clear chat"
              title="Clear chat history"
            >
              <svg className={styles.clearIcon} viewBox="0 0 24 24">
                <path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z" />
              </svg>
            </button>
          )}
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            <svg className={styles.closeIcon} viewBox="0 0 24 24">
              <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
            </svg>
          </button>
        </div>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.length === 0 && !isLoading && (
          <EmptyState />
        )}

        {messages.map((message, index) => (
          <Message key={index} message={message} />
        ))}

        {isLoading && <LoadingIndicator />}

        {error && (
          <div className={styles.errorMessage}>
            <span>{error}</span>
            {onRetry && (
              <button className={styles.retryButton} onClick={onRetry}>
                Retry
              </button>
            )}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <ChatInput onSend={onSend} disabled={isLoading} />
    </div>
  );
}

/**
 * Loading indicator with animated dots.
 */
function LoadingIndicator() {
  return (
    <div className={styles.loadingIndicator}>
      <div className={styles.loadingDots}>
        <div className={styles.loadingDot} />
        <div className={styles.loadingDot} />
        <div className={styles.loadingDot} />
      </div>
    </div>
  );
}

/**
 * Empty state when no messages.
 */
function EmptyState() {
  return (
    <div className={styles.emptyState}>
      <svg className={styles.emptyStateIcon} viewBox="0 0 24 24">
        <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      </svg>
      <div className={styles.emptyStateTitle}>Ask me anything!</div>
      <div className={styles.emptyStateText}>
        I can help you understand concepts from the Physical AI & Humanoid Robotics book.
      </div>
    </div>
  );
}
