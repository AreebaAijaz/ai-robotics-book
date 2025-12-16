import React from 'react';
import AskButton from './AskButton';
import useTextSelection from '../../hooks/useTextSelection';

/**
 * SelectionHandler component that detects text selection and shows
 * an "Ask about this" button when text is selected.
 */
export default function SelectionHandler({ onAskAboutSelection, children }) {
  const { selectedText, position, isVisible, clearSelection } = useTextSelection();

  const handleAskClick = () => {
    if (selectedText && onAskAboutSelection) {
      // Truncate very long selections
      const truncatedText = selectedText.length > 2000
        ? selectedText.slice(0, 2000) + '...'
        : selectedText;

      onAskAboutSelection(truncatedText);
      clearSelection();
    }
  };

  return (
    <>
      {children}
      {isVisible && selectedText && (
        <AskButton
          position={position}
          onClick={handleAskClick}
        />
      )}
    </>
  );
}
