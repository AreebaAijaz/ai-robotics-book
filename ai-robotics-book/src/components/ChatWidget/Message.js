import React from 'react';
import styles from './styles.module.css';

/**
 * Render a single chat message with optional citations.
 */
export default function Message({ message }) {
  const { role, content, citations } = message;
  const isUser = role === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageContent}>
        {isUser ? (
          <p>{content}</p>
        ) : (
          <MarkdownContent content={content} />
        )}
      </div>
      {!isUser && citations && citations.length > 0 && (
        <Citations citations={citations} />
      )}
    </div>
  );
}

/**
 * Simple markdown renderer for assistant messages.
 * Handles basic formatting: headers, bold, italic, code, lists.
 */
function MarkdownContent({ content }) {
  // Split content into paragraphs and render each
  const lines = content.split('\n');
  const elements = [];
  let currentList = [];
  let listType = null;
  let inCodeBlock = false;
  let codeContent = [];
  let codeLanguage = '';

  const flushList = () => {
    if (currentList.length > 0) {
      if (listType === 'ul') {
        elements.push(
          <ul key={`list-${elements.length}`}>
            {currentList.map((item, i) => (
              <li key={i}>{renderInline(item)}</li>
            ))}
          </ul>
        );
      } else {
        elements.push(
          <ol key={`list-${elements.length}`}>
            {currentList.map((item, i) => (
              <li key={i}>{renderInline(item)}</li>
            ))}
          </ol>
        );
      }
      currentList = [];
      listType = null;
    }
  };

  const flushCode = () => {
    if (codeContent.length > 0) {
      elements.push(
        <pre key={`code-${elements.length}`}>
          <code>{codeContent.join('\n')}</code>
        </pre>
      );
      codeContent = [];
      codeLanguage = '';
    }
  };

  lines.forEach((line, index) => {
    // Code block handling
    if (line.startsWith('```')) {
      if (inCodeBlock) {
        flushCode();
        inCodeBlock = false;
      } else {
        flushList();
        inCodeBlock = true;
        codeLanguage = line.slice(3).trim();
      }
      return;
    }

    if (inCodeBlock) {
      codeContent.push(line);
      return;
    }

    // Headers
    if (line.startsWith('### ')) {
      flushList();
      elements.push(<h4 key={index}>{renderInline(line.slice(4))}</h4>);
      return;
    }
    if (line.startsWith('## ')) {
      flushList();
      elements.push(<h3 key={index}>{renderInline(line.slice(3))}</h3>);
      return;
    }
    if (line.startsWith('# ')) {
      flushList();
      elements.push(<h2 key={index}>{renderInline(line.slice(2))}</h2>);
      return;
    }

    // Unordered list
    if (line.match(/^[-*]\s/)) {
      if (listType !== 'ul') {
        flushList();
        listType = 'ul';
      }
      currentList.push(line.slice(2));
      return;
    }

    // Ordered list
    if (line.match(/^\d+\.\s/)) {
      if (listType !== 'ol') {
        flushList();
        listType = 'ol';
      }
      currentList.push(line.replace(/^\d+\.\s/, ''));
      return;
    }

    // Empty line
    if (line.trim() === '') {
      flushList();
      return;
    }

    // Regular paragraph
    flushList();
    elements.push(<p key={index}>{renderInline(line)}</p>);
  });

  flushList();
  if (inCodeBlock) flushCode();

  return <>{elements}</>;
}

/**
 * Render inline formatting: bold, italic, code.
 */
function renderInline(text) {
  if (!text) return text;

  // Process inline code first
  const parts = text.split(/(`[^`]+`)/g);

  return parts.map((part, index) => {
    if (part.startsWith('`') && part.endsWith('`')) {
      return <code key={index}>{part.slice(1, -1)}</code>;
    }

    // Process bold and italic
    let processed = part;

    // Bold: **text**
    processed = processed.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');

    // Italic: *text* (but not inside **)
    processed = processed.replace(/(?<!\*)\*([^*]+)\*(?!\*)/g, '<em>$1</em>');

    if (processed !== part) {
      return <span key={index} dangerouslySetInnerHTML={{ __html: processed }} />;
    }

    return part;
  });
}

/**
 * Render citations list.
 */
function Citations({ citations }) {
  // Filter out duplicates and sort by relevance
  const uniqueCitations = citations
    .filter((c, i, arr) =>
      arr.findIndex(x => x.module === c.module && x.chapter === c.chapter) === i
    )
    .slice(0, 3); // Show top 3 citations

  return (
    <div className={styles.citations}>
      <div className={styles.citationsTitle}>Sources:</div>
      {uniqueCitations.map((citation, index) => (
        <div key={index} className={styles.citationItem}>
          <svg className={styles.citationIcon} viewBox="0 0 24 24">
            <path d="M14 2H6c-1.1 0-1.99.9-1.99 2L4 20c0 1.1.89 2 1.99 2H18c1.1 0 2-.9 2-2V8l-6-6zm2 16H8v-2h8v2zm0-4H8v-2h8v2zm-3-5V3.5L18.5 9H13z" />
          </svg>
          <span>
            {citation.module && citation.module !== 'General' ? `${citation.module} > ` : ''}
            {citation.chapter}
            {citation.section && ` > ${citation.section}`}
          </span>
        </div>
      ))}
    </div>
  );
}
