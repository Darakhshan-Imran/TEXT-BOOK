import React, { useState } from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from '@site/src/pages/AuthPage.module.css';

interface SignupFormProps {
  onSwitchToLogin: () => void;
}

const EyeIcon = () => (
  <svg className={styles.eyeIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
    <circle cx="12" cy="12" r="3" />
  </svg>
);

const EyeOffIcon = () => (
  <svg className={styles.eyeIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24" />
    <line x1="1" y1="1" x2="23" y2="23" />
  </svg>
);

const SignupForm: React.FC<SignupFormProps> = ({ onSwitchToLogin }) => {
  const { signUp } = useAuth();
  const docsUrl = useBaseUrl('/docs');
  const [email, setEmail] = useState<string>('');
  const [password, setPassword] = useState<string>('');
  const [showPassword, setShowPassword] = useState<boolean>(false);
  const [firstName, setFirstName] = useState<string>('');
  const [lastName, setLastName] = useState<string>('');
  const [error, setError] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);

  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>): Promise<void> => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await signUp(email, password, firstName, lastName);

      if (result.success) {
        setTimeout(() => {
          window.location.href = docsUrl;
        }, 100);
      } else {
        setError(result.error || 'Sign up failed');
        setLoading(false);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An unknown error occurred');
      setLoading(false);
    }
  };

  return (
    <div>
      <h2 className={styles.authTitle}>Sign Up</h2>
      {error && <div className={`${styles.alert} ${styles.alertDanger}`}>{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="first-name" className={styles.formLabel}>First Name</label>
          <input
            type="text"
            id="first-name"
            value={firstName}
            onChange={(e) => setFirstName(e.target.value)}
            className={styles.formControl}
            placeholder="Enter your first name"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="last-name" className={styles.formLabel}>Last Name</label>
          <input
            type="text"
            id="last-name"
            value={lastName}
            onChange={(e) => setLastName(e.target.value)}
            className={styles.formControl}
            placeholder="Enter your last name"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.formLabel}>Email</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            className={styles.formControl}
            placeholder="Enter your email"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password" className={styles.formLabel}>Password</label>
          <div className={styles.passwordWrapper}>
            <input
              type={showPassword ? 'text' : 'password'}
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              className={styles.formControl}
              placeholder="Create a password"
            />
            <button
              type="button"
              className={styles.eyeButton}
              onClick={() => setShowPassword(!showPassword)}
              aria-label={showPassword ? 'Hide password' : 'Show password'}
            >
              {showPassword ? <EyeOffIcon /> : <EyeIcon />}
            </button>
          </div>
        </div>

        <button
          type="submit"
          disabled={loading}
          className={styles.submitButton}
        >
          {loading ? 'Creating account...' : 'Sign Up'}
        </button>
      </form>

      <div className={styles.switchText}>
        <p>
          Already have an account?{' '}
          <a
            href="#"
            onClick={(e) => { e.preventDefault(); onSwitchToLogin(); }}
            className={styles.switchLink}
          >
            Login here
          </a>
        </p>
      </div>
    </div>
  );
};

export default SignupForm;
