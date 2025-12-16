import { useState, useEffect, useCallback } from 'react';

/**
 * Custom hook for detecting text selection on the page.
 * Returns the selected text and its position for displaying a floating button.
 */
export default function useTextSelection() {
  const [selection, setSelection] = useState({
    text: '',
    position: null,
    isVisible: false,
  });

  const handleSelectionChange = useCallback(() => {
    const windowSelection = window.getSelection();
    const selectedText = windowSelection?.toString().trim() || '';

    // Minimum selection length to show the button
    if (selectedText.length < 3) {
      setSelection({
        text: '',
        position: null,
        isVisible: false,
      });
      return;
    }

    // Get the bounding rect of the selection
    const range = windowSelection?.getRangeAt(0);
    if (!range) {
      setSelection({
        text: '',
        position: null,
        isVisible: false,
      });
      return;
    }

    const rect = range.getBoundingClientRect();

    // Calculate position for the floating button
    // Position it above the selection, centered horizontally
    const position = {
      top: rect.top + window.scrollY - 40, // 40px above selection
      left: rect.left + window.scrollX + rect.width / 2, // Centered
    };

    setSelection({
      text: selectedText,
      position,
      isVisible: true,
    });
  }, []);

  const clearSelection = useCallback(() => {
    setSelection({
      text: '',
      position: null,
      isVisible: false,
    });
    // Also clear the browser selection
    window.getSelection()?.removeAllRanges();
  }, []);

  useEffect(() => {
    // Listen for mouseup to detect selection completion
    const handleMouseUp = (e) => {
      // Small delay to ensure selection is complete
      setTimeout(handleSelectionChange, 10);
    };

    // Listen for touchend for mobile support
    const handleTouchEnd = (e) => {
      setTimeout(handleSelectionChange, 10);
    };

    // Clear selection when clicking elsewhere
    const handleMouseDown = (e) => {
      // Check if clicking on the ask button - if so, don't clear
      if (e.target.closest('[data-selection-button]')) {
        return;
      }
      // Small delay to allow for new selection
      setTimeout(() => {
        const selectedText = window.getSelection()?.toString().trim() || '';
        if (selectedText.length < 3) {
          clearSelection();
        }
      }, 10);
    };

    // Handle keyboard selection (Shift+Arrow keys)
    const handleKeyUp = (e) => {
      if (e.shiftKey) {
        setTimeout(handleSelectionChange, 10);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleTouchEnd);
    document.addEventListener('mousedown', handleMouseDown);
    document.addEventListener('keyup', handleKeyUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleTouchEnd);
      document.removeEventListener('mousedown', handleMouseDown);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleSelectionChange, clearSelection]);

  return {
    selectedText: selection.text,
    position: selection.position,
    isVisible: selection.isVisible,
    clearSelection,
  };
}
