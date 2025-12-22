/**
 * SignupForm - Registration form with 7-question background questionnaire.
 * Collects user credentials and profile data for personalized RAG responses.
 */
import React, { useState } from 'react';
import useAuth from '../../hooks/useAuth';
import styles from './styles.module.css';

// Questionnaire options based on spec.md
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
 * Multi-step signup form component.
 */
export default function SignupForm({ onSuccess, onSwitchToLogin }) {
  const { signUp, isLoading, error, clearError } = useAuth();

  // Form steps: credentials, questionnaire
  const [step, setStep] = useState(1);

  // Credentials state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  // Profile/questionnaire state
  const [profile, setProfile] = useState({
    expertise_level: '',
    interests: [],
    learning_goals: '',
    preferred_format: '',
    time_availability: '',
    challenges: '',
    device_preference: '',
  });

  // Validation errors
  const [validationErrors, setValidationErrors] = useState({});

  // Validate credentials step
  const validateCredentials = () => {
    const errors = {};

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!email) {
      errors.email = 'Email is required';
    } else if (!emailRegex.test(email)) {
      errors.email = 'Please enter a valid email address';
    }

    // Password validation (8+ chars, mixed case, number)
    if (!password) {
      errors.password = 'Password is required';
    } else if (password.length < 8) {
      errors.password = 'Password must be at least 8 characters';
    } else if (!/[a-z]/.test(password) || !/[A-Z]/.test(password)) {
      errors.password = 'Password must contain uppercase and lowercase letters';
    } else if (!/\d/.test(password)) {
      errors.password = 'Password must contain at least one number';
    }

    // Confirm password
    if (password !== confirmPassword) {
      errors.confirmPassword = 'Passwords do not match';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  // Validate questionnaire step
  const validateQuestionnaire = () => {
    const errors = {};

    if (!profile.expertise_level) {
      errors.expertise_level = 'Please select your expertise level';
    }
    if (profile.interests.length === 0) {
      errors.interests = 'Please select at least one interest';
    }
    if (!profile.learning_goals) {
      errors.learning_goals = 'Please select your learning goal';
    }
    if (!profile.preferred_format) {
      errors.preferred_format = 'Please select your preferred format';
    }
    if (!profile.time_availability) {
      errors.time_availability = 'Please select your time availability';
    }
    if (!profile.device_preference) {
      errors.device_preference = 'Please select your primary device';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  // Handle step 1 submit
  const handleCredentialsSubmit = (e) => {
    e.preventDefault();
    clearError();
    if (validateCredentials()) {
      setStep(2);
    }
  };

  // Handle interest toggle
  const toggleInterest = (value) => {
    setProfile((prev) => ({
      ...prev,
      interests: prev.interests.includes(value)
        ? prev.interests.filter((i) => i !== value)
        : [...prev.interests, value],
    }));
  };

  // Handle final submit
  const handleSubmit = async (e) => {
    e.preventDefault();
    clearError();

    if (!validateQuestionnaire()) {
      return;
    }

    const result = await signUp(email, password, profile);
    if (result.success && onSuccess) {
      onSuccess(result.user);
    }
  };

  // Handle back button
  const handleBack = () => {
    setStep(1);
    setValidationErrors({});
  };

  return (
    <div className={styles.signupForm}>
      {/* Progress indicator */}
      <div className={styles.progressBar}>
        <div className={`${styles.progressStep} ${step >= 1 ? styles.active : ''}`}>
          <span className={styles.stepNumber}>1</span>
          <span className={styles.stepLabel}>Account</span>
        </div>
        <div className={styles.progressLine} />
        <div className={`${styles.progressStep} ${step >= 2 ? styles.active : ''}`}>
          <span className={styles.stepNumber}>2</span>
          <span className={styles.stepLabel}>Profile</span>
        </div>
      </div>

      {/* Step 1: Credentials */}
      {step === 1 && (
        <form onSubmit={handleCredentialsSubmit} className={styles.form}>
          <h2 className={styles.title}>Create Account</h2>
          <p className={styles.subtitle}>
            Sign up to access the personalized AI chatbot
          </p>

          <div className={styles.field}>
            <label htmlFor="email" className={styles.label}>Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className={`${styles.input} ${validationErrors.email ? styles.inputError : ''}`}
              placeholder="you@example.com"
              autoComplete="email"
              autoFocus
            />
            {validationErrors.email && (
              <span className={styles.errorText}>{validationErrors.email}</span>
            )}
          </div>

          <div className={styles.field}>
            <label htmlFor="password" className={styles.label}>Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className={`${styles.input} ${validationErrors.password ? styles.inputError : ''}`}
              placeholder="Min 8 chars, mixed case, number"
              autoComplete="new-password"
            />
            {validationErrors.password && (
              <span className={styles.errorText}>{validationErrors.password}</span>
            )}
          </div>

          <div className={styles.field}>
            <label htmlFor="confirmPassword" className={styles.label}>Confirm Password</label>
            <input
              type="password"
              id="confirmPassword"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              className={`${styles.input} ${validationErrors.confirmPassword ? styles.inputError : ''}`}
              placeholder="Re-enter password"
              autoComplete="new-password"
            />
            {validationErrors.confirmPassword && (
              <span className={styles.errorText}>{validationErrors.confirmPassword}</span>
            )}
          </div>

          <button type="submit" className={styles.submitButton}>
            Continue
          </button>

          <p className={styles.switchText}>
            Already have an account?{' '}
            <button type="button" onClick={onSwitchToLogin} className={styles.linkButton}>
              Log in
            </button>
          </p>
        </form>
      )}

      {/* Step 2: Questionnaire */}
      {step === 2 && (
        <form onSubmit={handleSubmit} className={styles.form}>
          <h2 className={styles.title}>Tell Us About Yourself</h2>
          <p className={styles.subtitle}>
            Help us personalize your learning experience
          </p>

          {error && <div className={styles.errorBanner}>{error}</div>}

          {/* Q1: Expertise Level */}
          <div className={styles.field}>
            <label className={styles.label}>What is your expertise level in robotics/AI?</label>
            <div className={styles.radioGroup}>
              {EXPERTISE_LEVELS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="expertise_level"
                    value={option.value}
                    checked={profile.expertise_level === option.value}
                    onChange={(e) => setProfile({ ...profile, expertise_level: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.expertise_level && (
              <span className={styles.errorText}>{validationErrors.expertise_level}</span>
            )}
          </div>

          {/* Q2: Interests (multi-select) */}
          <div className={styles.field}>
            <label className={styles.label}>Which topics interest you most? (Select all that apply)</label>
            <div className={styles.checkboxGroup}>
              {INTERESTS.map((option) => (
                <label key={option.value} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={profile.interests.includes(option.value)}
                    onChange={() => toggleInterest(option.value)}
                    className={styles.checkbox}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.interests && (
              <span className={styles.errorText}>{validationErrors.interests}</span>
            )}
          </div>

          {/* Q3: Learning Goals */}
          <div className={styles.field}>
            <label className={styles.label}>What is your learning goal?</label>
            <div className={styles.radioGroup}>
              {LEARNING_GOALS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="learning_goals"
                    value={option.value}
                    checked={profile.learning_goals === option.value}
                    onChange={(e) => setProfile({ ...profile, learning_goals: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.learning_goals && (
              <span className={styles.errorText}>{validationErrors.learning_goals}</span>
            )}
          </div>

          {/* Q4: Preferred Format */}
          <div className={styles.field}>
            <label className={styles.label}>How do you prefer to learn?</label>
            <div className={styles.radioGroup}>
              {PREFERRED_FORMATS.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="preferred_format"
                    value={option.value}
                    checked={profile.preferred_format === option.value}
                    onChange={(e) => setProfile({ ...profile, preferred_format: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.preferred_format && (
              <span className={styles.errorText}>{validationErrors.preferred_format}</span>
            )}
          </div>

          {/* Q5: Time Availability */}
          <div className={styles.field}>
            <label className={styles.label}>How much time do you typically have for learning?</label>
            <div className={styles.radioGroup}>
              {TIME_AVAILABILITY.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="time_availability"
                    value={option.value}
                    checked={profile.time_availability === option.value}
                    onChange={(e) => setProfile({ ...profile, time_availability: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.time_availability && (
              <span className={styles.errorText}>{validationErrors.time_availability}</span>
            )}
          </div>

          {/* Q6: Current Challenges (optional) */}
          <div className={styles.field}>
            <label htmlFor="challenges" className={styles.label}>
              What challenges are you facing? (Optional)
            </label>
            <textarea
              id="challenges"
              value={profile.challenges}
              onChange={(e) => setProfile({ ...profile, challenges: e.target.value })}
              className={styles.textarea}
              placeholder="E.g., Understanding control theory, implementing SLAM..."
              rows={3}
            />
          </div>

          {/* Q7: Device Preference */}
          <div className={styles.field}>
            <label className={styles.label}>What device do you primarily use?</label>
            <div className={styles.radioGroup}>
              {DEVICE_PREFERENCE.map((option) => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="device_preference"
                    value={option.value}
                    checked={profile.device_preference === option.value}
                    onChange={(e) => setProfile({ ...profile, device_preference: e.target.value })}
                    className={styles.radio}
                  />
                  <span>{option.label}</span>
                </label>
              ))}
            </div>
            {validationErrors.device_preference && (
              <span className={styles.errorText}>{validationErrors.device_preference}</span>
            )}
          </div>

          <div className={styles.buttonGroup}>
            <button type="button" onClick={handleBack} className={styles.backButton}>
              Back
            </button>
            <button type="submit" className={styles.submitButton} disabled={isLoading}>
              {isLoading ? 'Creating Account...' : 'Create Account'}
            </button>
          </div>
        </form>
      )}
    </div>
  );
}
