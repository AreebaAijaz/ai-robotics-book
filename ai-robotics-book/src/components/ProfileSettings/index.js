/**
 * ProfileSettings - View and edit user profile questionnaire answers.
 * Allows users to update their learning preferences for personalized RAG.
 */
import React, { useState, useEffect } from 'react';
import useProfile from '../../hooks/useProfile';
import useAuth from '../../hooks/useAuth';
import styles from './styles.module.css';

// Questionnaire options (same as SignupForm)
const EXPERTISE_LEVELS = [
  { value: 'beginner', label: 'Beginner - New to robotics/AI' },
  { value: 'intermediate', label: 'Intermediate - Some experience' },
  { value: 'advanced', label: 'Advanced - Professional experience' },
  { value: 'expert', label: 'Expert - Research/Industry leader' },
];

const INTERESTS = [
  { value: 'humanoid_design', label: 'Humanoid Robot Design' },
  { value: 'locomotion', label: 'Locomotion & Movement' },
  { value: 'manipulation', label: 'Manipulation & Grasping' },
  { value: 'perception', label: 'Perception & Sensing' },
  { value: 'ai_integration', label: 'AI & Machine Learning' },
  { value: 'control_systems', label: 'Control Systems' },
  { value: 'hri', label: 'Human-Robot Interaction' },
  { value: 'ethics', label: 'Ethics & Safety' },
];

const LEARNING_GOALS = [
  { value: 'casual', label: 'Casual reading - General interest' },
  { value: 'deep_understanding', label: 'Deep understanding - Learn thoroughly' },
  { value: 'practical', label: 'Practical application - Build projects' },
  { value: 'research', label: 'Research - Academic or professional' },
];

const PREFERRED_FORMATS = [
  { value: 'text', label: 'Text explanations' },
  { value: 'diagrams', label: 'Visual diagrams' },
  { value: 'videos', label: 'Video content' },
  { value: 'code', label: 'Code examples' },
  { value: 'interactive', label: 'Interactive demos' },
];

const TIME_AVAILABILITY = [
  { value: 'quick_5min', label: '5 minutes - Quick answers' },
  { value: 'medium_15min', label: '15 minutes - Explanations' },
  { value: 'deep_30min', label: '30+ minutes - Deep dives' },
];

const DEVICE_PREFERENCE = [
  { value: 'desktop', label: 'Desktop/Laptop' },
  { value: 'tablet', label: 'Tablet' },
  { value: 'mobile', label: 'Mobile phone' },
];

/**
 * ProfileSettings component for viewing and editing user profile.
 * @param {Object} props
 * @param {Function} props.onClose - Callback when settings should close
 */
export default function ProfileSettings({ onClose }) {
  const { user } = useAuth();
  const { profile, isLoading, error, updateProfile, refetch } = useProfile();

  // Edit mode state
  const [isEditing, setIsEditing] = useState(false);

  // Form state for editing
  const [formData, setFormData] = useState({
    expertise_level: '',
    interests: [],
    learning_goals: '',
    preferred_format: '',
    time_availability: '',
    challenges: '',
    device_preference: '',
  });

  // Save state
  const [isSaving, setIsSaving] = useState(false);
  const [saveError, setSaveError] = useState(null);
  const [saveSuccess, setSaveSuccess] = useState(false);

  // Sync form data with profile
  useEffect(() => {
    if (profile) {
      setFormData({
        expertise_level: profile.expertise_level || '',
        interests: profile.interests || [],
        learning_goals: profile.learning_goals || '',
        preferred_format: profile.preferred_format || '',
        time_availability: profile.time_availability || '',
        challenges: profile.challenges || '',
        device_preference: profile.device_preference || '',
      });
    }
  }, [profile]);

  // Handle interest toggle
  const toggleInterest = (value) => {
    setFormData((prev) => ({
      ...prev,
      interests: prev.interests.includes(value)
        ? prev.interests.filter((i) => i !== value)
        : [...prev.interests, value],
    }));
  };

  // Start editing
  const handleEdit = () => {
    setIsEditing(true);
    setSaveError(null);
    setSaveSuccess(false);
  };

  // Cancel editing
  const handleCancel = () => {
    // Reset form to current profile
    if (profile) {
      setFormData({
        expertise_level: profile.expertise_level || '',
        interests: profile.interests || [],
        learning_goals: profile.learning_goals || '',
        preferred_format: profile.preferred_format || '',
        time_availability: profile.time_availability || '',
        challenges: profile.challenges || '',
        device_preference: profile.device_preference || '',
      });
    }
    setIsEditing(false);
    setSaveError(null);
  };

  // Save changes
  const handleSave = async () => {
    setIsSaving(true);
    setSaveError(null);
    setSaveSuccess(false);

    const result = await updateProfile(formData);

    setIsSaving(false);

    if (result.success) {
      setSaveSuccess(true);
      setIsEditing(false);
      // Clear success message after 3 seconds
      setTimeout(() => setSaveSuccess(false), 3000);
    } else {
      setSaveError(result.error || 'Failed to save changes');
    }
  };

  // Get label for a value
  const getLabel = (options, value) => {
    const option = options.find((o) => o.value === value);
    return option ? option.label : value;
  };

  // Loading state
  if (isLoading && !profile) {
    return (
      <div className={styles.profileSettings}>
        <div className={styles.header}>
          <h2 className={styles.title}>Profile Settings</h2>
          {onClose && (
            <button onClick={onClose} className={styles.closeButton}>
              &times;
            </button>
          )}
        </div>
        <div className={styles.loading}>Loading profile...</div>
      </div>
    );
  }

  // Error state
  if (error && !profile) {
    return (
      <div className={styles.profileSettings}>
        <div className={styles.header}>
          <h2 className={styles.title}>Profile Settings</h2>
          {onClose && (
            <button onClick={onClose} className={styles.closeButton}>
              &times;
            </button>
          )}
        </div>
        <div className={styles.errorBanner}>
          {error}
          <button onClick={refetch} className={styles.retryButton}>
            Retry
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.profileSettings}>
      <div className={styles.header}>
        <h2 className={styles.title}>Profile Settings</h2>
        {onClose && (
          <button onClick={onClose} className={styles.closeButton}>
            &times;
          </button>
        )}
      </div>

      {/* User info */}
      <div className={styles.userInfo}>
        <div className={styles.avatar}>
          {user?.email?.[0]?.toUpperCase() || 'U'}
        </div>
        <div className={styles.userDetails}>
          <span className={styles.userEmail}>{user?.email}</span>
          <span className={styles.memberSince}>
            Member since {new Date(user?.created_at || Date.now()).toLocaleDateString()}
          </span>
        </div>
      </div>

      {/* Success message */}
      {saveSuccess && (
        <div className={styles.successBanner}>
          Profile updated successfully!
        </div>
      )}

      {/* Save error */}
      {saveError && (
        <div className={styles.errorBanner}>{saveError}</div>
      )}

      {/* Profile fields */}
      <div className={styles.form}>
        {/* Expertise Level */}
        <div className={styles.field}>
          <label className={styles.label}>Expertise Level</label>
          {isEditing ? (
            <div className={styles.radioGroup}>
              {EXPERTISE_LEVELS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="expertise_level"
                    value={option.value}
                    checked={formData.expertise_level === option.value}
                    onChange={(e) => setFormData({ ...formData, expertise_level: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.value}>
              {getLabel(EXPERTISE_LEVELS, formData.expertise_level)}
            </div>
          )}
        </div>

        {/* Interests */}
        <div className={styles.field}>
          <label className={styles.label}>Topics of Interest</label>
          {isEditing ? (
            <div className={styles.checkboxGroup}>
              {INTERESTS.map((option) => (
                <label key={option.value} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={formData.interests.includes(option.value)}
                    onChange={() => toggleInterest(option.value)}
                    className={styles.checkbox}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.tagList}>
              {formData.interests.map((interest) => (
                <span key={interest} className={styles.tag}>
                  {getLabel(INTERESTS, interest)}
                </span>
              ))}
            </div>
          )}
        </div>

        {/* Learning Goals */}
        <div className={styles.field}>
          <label className={styles.label}>Learning Goal</label>
          {isEditing ? (
            <div className={styles.radioGroup}>
              {LEARNING_GOALS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="learning_goals"
                    value={option.value}
                    checked={formData.learning_goals === option.value}
                    onChange={(e) => setFormData({ ...formData, learning_goals: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.value}>
              {getLabel(LEARNING_GOALS, formData.learning_goals)}
            </div>
          )}
        </div>

        {/* Preferred Format */}
        <div className={styles.field}>
          <label className={styles.label}>Preferred Learning Format</label>
          {isEditing ? (
            <div className={styles.radioGroup}>
              {PREFERRED_FORMATS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="preferred_format"
                    value={option.value}
                    checked={formData.preferred_format === option.value}
                    onChange={(e) => setFormData({ ...formData, preferred_format: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.value}>
              {getLabel(PREFERRED_FORMATS, formData.preferred_format)}
            </div>
          )}
        </div>

        {/* Time Availability */}
        <div className={styles.field}>
          <label className={styles.label}>Time Availability</label>
          {isEditing ? (
            <div className={styles.radioGroup}>
              {TIME_AVAILABILITY.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="time_availability"
                    value={option.value}
                    checked={formData.time_availability === option.value}
                    onChange={(e) => setFormData({ ...formData, time_availability: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.value}>
              {getLabel(TIME_AVAILABILITY, formData.time_availability)}
            </div>
          )}
        </div>

        {/* Challenges */}
        <div className={styles.field}>
          <label className={styles.label}>Current Challenges</label>
          {isEditing ? (
            <textarea
              value={formData.challenges}
              onChange={(e) => setFormData({ ...formData, challenges: e.target.value })}
              className={styles.textarea}
              placeholder="E.g., Understanding control theory, implementing SLAM..."
              rows={3}
            />
          ) : (
            <div className={styles.value}>
              {formData.challenges || <span className={styles.empty}>Not specified</span>}
            </div>
          )}
        </div>

        {/* Device Preference */}
        <div className={styles.field}>
          <label className={styles.label}>Primary Device</label>
          {isEditing ? (
            <div className={styles.radioGroup}>
              {DEVICE_PREFERENCE.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="device_preference"
                    value={option.value}
                    checked={formData.device_preference === option.value}
                    onChange={(e) => setFormData({ ...formData, device_preference: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
          ) : (
            <div className={styles.value}>
              {getLabel(DEVICE_PREFERENCE, formData.device_preference)}
            </div>
          )}
        </div>
      </div>

      {/* Action buttons */}
      <div className={styles.actions}>
        {isEditing ? (
          <>
            <button
              onClick={handleCancel}
              className={styles.cancelButton}
              disabled={isSaving}
            >
              Cancel
            </button>
            <button
              onClick={handleSave}
              className={styles.saveButton}
              disabled={isSaving}
            >
              {isSaving ? 'Saving...' : 'Save Changes'}
            </button>
          </>
        ) : (
          <button onClick={handleEdit} className={styles.editButton}>
            Edit Profile
          </button>
        )}
      </div>
    </div>
  );
}
