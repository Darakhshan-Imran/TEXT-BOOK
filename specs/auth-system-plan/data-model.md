# Data Model: Authentication System

## User Entity

### Attributes
- **id** (UUID/Integer): Primary key, unique identifier for the user
- **email** (String): User's email address, must be unique
- **hashed_password** (String): Bcrypt hashed password (nullable for OAuth users)
- **first_name** (String): User's first name (optional)
- **last_name** (String): User's last name (optional)
- **is_active** (Boolean): Whether the account is active, defaults to True
- **is_verified** (Boolean): Whether the email is verified, defaults to False
- **auth_provider** (String): Provider used for authentication (email, google, github)
- **provider_id** (String): Provider's unique ID for the user (for OAuth users)
- **created_at** (DateTime): Timestamp when user was created
- **updated_at** (DateTime): Timestamp when user was last updated

### Validation Rules
- Email must follow standard email format
- Email must be unique across all users
- Password must meet minimum security requirements (if provided)
- auth_provider must be one of: 'email', 'google', 'github'

### Relationships
- One-to-Many: User to Sessions (one user can have multiple active sessions)

## Session Entity

### Attributes
- **id** (UUID): Primary key, unique session identifier
- **user_id** (UUID/Integer): Foreign key referencing User
- **token** (String): JWT token value (hashed for security)
- **expires_at** (DateTime): When the session expires
- **created_at** (DateTime): When the session was created
- **last_accessed** (DateTime): When the session was last used
- **device_info** (String): Information about the device used (optional)
- **ip_address** (String): IP address of the session (optional)

### Validation Rules
- Token must be unique
- expires_at must be in the future
- Session must be linked to an active user

### Relationships
- Many-to-One: Session to User (many sessions belong to one user)

## OAuthAccount Entity (Optional)

### Attributes
- **id** (UUID): Primary key
- **user_id** (UUID/Integer): Foreign key referencing User
- **provider** (String): OAuth provider name (google, github)
- **provider_account_id** (String): Provider's account ID
- **provider_account_email** (String): Email from provider
- **created_at** (DateTime): When the OAuth link was created

### Validation Rules
- Combination of provider and provider_account_id must be unique
- Must be linked to an existing user

### Relationships
- Many-to-One: OAuthAccount to User