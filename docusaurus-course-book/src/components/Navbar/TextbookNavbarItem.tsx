import React from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import NavbarItem from '@theme/NavbarItem';

interface TextbookNavbarItemProps {
  [key: string]: any;
}

const TextbookNavbarItem: React.FC<TextbookNavbarItemProps> = (props) => {
  const { user, isLoading } = useAuth();

  // Don't show anything while loading
  if (isLoading) {
    return null;
  }

  // Only show Textbook link if user is logged in
  if (!user) {
    return null;
  }

  return (
    <NavbarItem
      {...props}
      type="doc"
      docId="intro"
      position="left"
      label="Textbook"
    />
  );
};

export default TextbookNavbarItem;
