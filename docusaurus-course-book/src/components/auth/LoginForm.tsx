import React, { useState } from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface LoginFormProps {
  onSwitchToSignup: () => void;
}

const LoginForm: React.FC<LoginFormProps> = ({ onSwitchToSignup }) => {
  const { signIn } = useAuth();
  const docsUrl = useBaseUrl('/docs');
  const [email, setEmail] = useState<string>('');
  const [password, setPassword] = useState<string>('');
  const [error, setError] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);

  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>): Promise<void> => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await signIn(email, password);

      if (result.success) {
        // Redirect to docs after successful login
        setTimeout(() => {
          window.location.href = docsUrl;
        }, 100);
      } else {
        setError(result.error || 'Login failed');
        setLoading(false);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An unknown error occurred');
      setLoading(false);
    }
  };

  return (
    <div>
      <h2>Login</h2>
      {error && <div className="alert alert--danger">{error}</div>}

      <form onSubmit={handleSubmit}>
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
          {loading ? 'Logging in...' : 'Login'}
        </button>
      </form>

      <div className="margin-top--lg">
        <p>
          Don't have an account?{' '}
          <a href="#" onClick={(e) => { e.preventDefault(); onSwitchToSignup(); }}>
            Sign up here
          </a>
        </p>
      </div>
    </div>
  );
};

export default LoginForm;