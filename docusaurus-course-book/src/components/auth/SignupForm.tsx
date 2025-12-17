import React, { useState } from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface SignupFormProps {
  onSwitchToLogin: () => void;
}

const SignupForm: React.FC<SignupFormProps> = ({ onSwitchToLogin }) => {
  const { signUp } = useAuth();
  const docsUrl = useBaseUrl('/docs');
  const [email, setEmail] = useState<string>('');
  const [password, setPassword] = useState<string>('');
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
        // Redirect to docs after successful signup
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
      <h2>Sign Up</h2>
      {error && <div className="alert alert--danger">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="margin-bottom--lg">
          <label htmlFor="first-name">First Name:</label>
          <input
            type="text"
            id="first-name"
            value={firstName}
            onChange={(e) => setFirstName(e.target.value)}
            className="form-control"
          />
        </div>

        <div className="margin-bottom--lg">
          <label htmlFor="last-name">Last Name:</label>
          <input
            type="text"
            id="last-name"
            value={lastName}
            onChange={(e) => setLastName(e.target.value)}
            className="form-control"
          />
        </div>

        <div className="margin-bottom--lg">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            className="form-control"
          />
        </div>

        <div className="margin-bottom--lg">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            className="form-control"
          />
        </div>

        <button
          type="submit"
          disabled={loading}
          className={`button button--primary button--block${loading ? ' disabled' : ''}`}
        >
          {loading ? 'Creating account...' : 'Sign Up'}
        </button>
      </form>

      <div className="margin-top--lg">
        <p>
          Already have an account?{' '}
          <a href="#" onClick={(e) => { e.preventDefault(); onSwitchToLogin(); }}>
            Login here
          </a>
        </p>
      </div>
    </div>
  );
};

export default SignupForm;