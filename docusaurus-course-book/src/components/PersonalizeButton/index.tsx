import React, { useState, useCallback, useEffect } from 'react';
import { useAuth } from '@site/src/auth/auth-context';
import styles from './styles.module.css';

type PersonalizationMode = 'original' | 'simplified' | 'expanded' | 'editing';

// English simplifications
const englishSimplifications: Record<string, string> = {
  'utilize': 'use',
  'implement': 'build',
  'functionality': 'feature',
  'configuration': 'setup',
  'initialization': 'startup',
  'instantiate': 'create',
  'methodology': 'method',
  'comprehensive': 'complete',
  'subsequently': 'then',
  'fundamental': 'basic',
  'demonstrate': 'show',
  'facilitate': 'help',
  'approximately': 'about',
  'modifications': 'changes',
  'parameters': 'settings',
  'autonomous': 'self-running',
  'perception': 'sensing',
  'manipulation': 'handling',
  'kinematics': 'motion math',
  'dynamics': 'force and motion',
};

// Urdu simplifications (technical terms → simpler Urdu)
const urduSimplifications: Record<string, string> = {
  'خودمختار': 'خود چلنے والا',
  'ادراک': 'سمجھ',
  'ہیرا پھیری': 'سنبھالنا',
  'حرکیات': 'حرکت کا حساب',
  'متحرک': 'چلتا پھرتا',
  'تشکیل': 'بناوٹ',
  'ماحولیات': 'ارد گرد',
  'سینسر': 'محسوس کرنے والا آلہ',
  'الگورتھم': 'طریقہ کار',
  'انٹرفیس': 'رابطہ',
  'پیرامیٹرز': 'ترتیبات',
  'کنفیگریشن': 'ترتیب',
  'simulation': 'نقل',
  'روبوٹکس': 'روبوٹ سازی',
  'navigation': 'راستہ تلاش',
  'architecture': 'ڈھانچہ',
  'infrastructure': 'بنیادی ڈھانچہ',
  'implementation': 'عمل درآمد',
  'functionality': 'کام',
  'artificial intelligence': 'مصنوعی ذہانت',
  'machine learning': 'مشین سیکھنا',
};

// Technical terms for expansion (works for both languages)
const technicalTermsEnglish: Record<string, string> = {
  'ROS 2': 'Robot Operating System 2 - A framework for building robot applications',
  'URDF': 'Unified Robot Description Format - XML format to describe robot structure',
  'Gazebo': 'A powerful 3D robot simulator',
  'Isaac Sim': 'NVIDIA\'s photorealistic robot simulator',
  'Nav2': 'Navigation 2 - ROS 2 navigation stack for autonomous robots',
  'LiDAR': 'Light Detection and Ranging - Sensor that measures distances using laser',
  'SLAM': 'Simultaneous Localization and Mapping - Building maps while tracking position',
  'IMU': 'Inertial Measurement Unit - Sensor measuring acceleration and rotation',
  'TensorRT': 'NVIDIA\'s deep learning inference optimizer',
  'GPU': 'Graphics Processing Unit - Parallel processor for graphics and AI',
  'API': 'Application Programming Interface - Way for programs to communicate',
  'SDK': 'Software Development Kit - Tools for building applications',
};

const technicalTermsUrdu: Record<string, string> = {
  'ROS 2': 'روبوٹ آپریٹنگ سسٹم 2 - روبوٹ ایپلی کیشنز بنانے کا فریم ورک',
  'URDF': 'یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ - روبوٹ کی ساخت بیان کرنے کا XML فارمیٹ',
  'Gazebo': 'ایک طاقتور 3D روبوٹ سمولیٹر',
  'Isaac Sim': 'NVIDIA کا حقیقت پسندانہ روبوٹ سمولیٹر',
  'LiDAR': 'لائٹ ڈٹیکشن اینڈ رینجنگ - لیزر سے فاصلہ ناپنے والا سینسر',
  'SLAM': 'بیک وقت مقام اور نقشہ بندی - پوزیشن ٹریک کرتے ہوئے نقشے بنانا',
  'IMU': 'انرشیئل میژرمنٹ یونٹ - ایکسلریشن اور گردش ناپنے والا سینسر',
  'GPU': 'گرافکس پروسیسنگ یونٹ - گرافکس اور AI کے لیے پیرالل پروسیسر',
  'API': 'ایپلی کیشن پروگرامنگ انٹرفیس - پروگراموں کے درمیان رابطے کا طریقہ',
  'SDK': 'سافٹ ویئر ڈویلپمنٹ کٹ - ایپلی کیشنز بنانے کے اوزار',
};

const PersonalizeButton: React.FC = () => {
  const { user } = useAuth();
  const [mode, setMode] = useState<PersonalizationMode>('original');
  const [isProcessing, setIsProcessing] = useState(false);
  const [originalContent, setOriginalContent] = useState<string>('');
  const [isEditing, setIsEditing] = useState(false);
  const [showDownloadMenu, setShowDownloadMenu] = useState(false);
  const [currentLocale, setCurrentLocale] = useState<string>('en');

  // Detect current locale from URL
  useEffect(() => {
    const path = window.location.pathname;
    if (path.includes('/ur/')) {
      setCurrentLocale('ur');
    } else if (path.includes('/zh/')) {
      setCurrentLocale('zh');
    } else if (path.includes('/fr/')) {
      setCurrentLocale('fr');
    } else {
      setCurrentLocale('en');
    }
  }, []);

  // Get the appropriate simplification dictionary based on locale
  const getSimplifications = () => {
    return currentLocale === 'ur' ? urduSimplifications : englishSimplifications;
  };

  // Get the appropriate technical terms based on locale
  const getTechnicalTerms = () => {
    return currentLocale === 'ur' ? technicalTermsUrdu : technicalTermsEnglish;
  };

  // UI labels based on locale
  const labels = currentLocale === 'ur' ? {
    personalizeContent: 'مواد کو ذاتی بنائیں',
    simplify: 'آسان بنائیں',
    expand: 'تفصیل دکھائیں',
    edit: 'ترمیم کریں',
    doneEditing: 'ترمیم مکمل',
    download: 'ڈاؤن لوڈ',
    downloadMD: 'مارک ڈاؤن (.md) ڈاؤن لوڈ کریں',
    downloadPDF: 'پی ڈی ایف (.pdf) ڈاؤن لوڈ کریں',
    restore: 'واپس لائیں',
    simplifiedView: '✓ آسان نظارہ',
    expandedView: '✓ تفصیلی نظارہ',
    editMode: '✏️ ترمیم موڈ - متن پر کلک کریں',
  } : {
    personalizeContent: 'Personalize Content',
    simplify: 'Simplify',
    expand: 'Expand',
    edit: 'Edit',
    doneEditing: 'Done Editing',
    download: 'Download',
    downloadMD: 'Download as Markdown (.md)',
    downloadPDF: 'Download as PDF (.pdf)',
    restore: 'Restore',
    simplifiedView: '✓ Simplified view',
    expandedView: '✓ Expanded with tooltips',
    editMode: '✏️ Edit mode - Click on text to edit',
  };

  const getContentElement = (): Element | null => {
    const selectors = [
      '.theme-doc-markdown',
      'article .markdown',
      'article',
    ];

    for (const selector of selectors) {
      const element = document.querySelector(selector);
      if (element) return element;
    }
    return null;
  };

  const getChapterTitle = (): string => {
    const titleElement = document.querySelector('h1');
    return titleElement?.textContent || 'Chapter';
  };

  const simplifyContent = async (text: string, forHTML: boolean = false): Promise<string> => {
    const simplifications = getSimplifications();
    const isUrdu = currentLocale === 'ur';

    let simplified = text;
    for (const [complex, simple] of Object.entries(simplifications)) {
      // For Urdu, don't use word boundaries as they don't work well with Arabic script
      const regex = isUrdu
        ? new RegExp(complex, 'gi')
        : new RegExp(`\\b${complex}\\b`, 'gi');

      if (forHTML) {
        // Wrap simplified word in highlighted span
        const tooltipText = isUrdu ? `اصل: ${complex}` : `Original: ${complex}`;
        simplified = simplified.replace(regex, `<span class="${styles.simplified}" title="${tooltipText}">${simple}</span>`);
      } else {
        simplified = simplified.replace(regex, simple);
      }
    }

    return simplified;
  };

  const expandContent = async (element: Element): Promise<void> => {
    const technicalTerms = getTechnicalTerms();

    const walker = document.createTreeWalker(
      element,
      NodeFilter.SHOW_TEXT,
      null
    );

    const textNodes: Text[] = [];
    let node: Text | null;
    while ((node = walker.nextNode() as Text)) {
      if (node.parentElement?.closest('pre, code, script, style')) continue;
      textNodes.push(node);
    }

    for (const textNode of textNodes) {
      let html = textNode.textContent || '';
      let hasChanges = false;

      for (const [term, explanation] of Object.entries(technicalTerms)) {
        const regex = new RegExp(`\\b(${term})\\b`, 'g');
        if (regex.test(html)) {
          html = html.replace(regex, `<span class="${styles.tooltip}" data-tooltip="${explanation}">$1</span>`);
          hasChanges = true;
        }
      }

      if (hasChanges) {
        const span = document.createElement('span');
        span.innerHTML = html;
        textNode.parentNode?.replaceChild(span, textNode);
      }
    }
  };

  const handleSimplify = useCallback(async () => {
    const contentElement = getContentElement();
    if (!contentElement) return;

    setIsProcessing(true);

    try {
      if (mode === 'original') {
        setOriginalContent(contentElement.innerHTML);
      }

      const textElements = contentElement.querySelectorAll('p, li, h1, h2, h3, h4, h5, h6, td, th, blockquote');

      for (const el of textElements) {
        if (el.closest('pre') || el.closest('code')) continue;

        const originalText = el.textContent || '';
        const simplifiedHTML = await simplifyContent(originalText, true);

        // Only update if there were actual simplifications (contains highlighted spans)
        if (simplifiedHTML.includes(styles.simplified)) {
          el.innerHTML = simplifiedHTML;
        }
      }

      setMode('simplified');
      setIsEditing(false);
    } catch (error) {
      console.error('Simplification error:', error);
    } finally {
      setIsProcessing(false);
    }
  }, [mode]);

  const handleExpand = useCallback(async () => {
    const contentElement = getContentElement();
    if (!contentElement) return;

    setIsProcessing(true);

    try {
      if (mode === 'original') {
        setOriginalContent(contentElement.innerHTML);
      }

      await expandContent(contentElement);
      setMode('expanded');
      setIsEditing(false);
    } catch (error) {
      console.error('Expansion error:', error);
    } finally {
      setIsProcessing(false);
    }
  }, [mode]);

  const handleEdit = useCallback(() => {
    const contentElement = getContentElement();
    if (!contentElement) return;

    if (!isEditing) {
      if (!originalContent) {
        setOriginalContent(contentElement.innerHTML);
      }

      const editableElements = contentElement.querySelectorAll('p, li, h1, h2, h3, h4, h5, h6, td, th, blockquote');
      editableElements.forEach((el) => {
        if (!el.closest('pre') && !el.closest('code')) {
          (el as HTMLElement).contentEditable = 'true';
          (el as HTMLElement).classList.add(styles.editable);
        }
      });

      setIsEditing(true);
      setMode('editing');
    } else {
      const editableElements = contentElement.querySelectorAll('[contenteditable="true"]');
      editableElements.forEach((el) => {
        (el as HTMLElement).contentEditable = 'false';
        (el as HTMLElement).classList.remove(styles.editable);
      });

      setIsEditing(false);
      setMode('original');
    }
  }, [isEditing, originalContent]);

  const handleDownloadMD = useCallback(() => {
    const contentElement = getContentElement();
    if (!contentElement) return;

    const title = getChapterTitle();
    const date = new Date().toLocaleDateString();

    let textContent = `${title}\n`;
    textContent += `${'='.repeat(title.length)}\n`;
    textContent += `Downloaded on: ${date}\n`;
    textContent += `Edited by: ${user?.name || user?.email || 'User'}\n\n`;
    textContent += '---\n\n';

    const elements = contentElement.querySelectorAll('h1, h2, h3, h4, h5, h6, p, li, pre, blockquote');
    elements.forEach((el) => {
      const tagName = el.tagName.toLowerCase();
      const text = el.textContent?.trim() || '';

      if (!text) return;

      switch (tagName) {
        case 'h1':
          textContent += `\n# ${text}\n\n`;
          break;
        case 'h2':
          textContent += `\n## ${text}\n\n`;
          break;
        case 'h3':
          textContent += `\n### ${text}\n\n`;
          break;
        case 'h4':
        case 'h5':
        case 'h6':
          textContent += `\n#### ${text}\n\n`;
          break;
        case 'li':
          textContent += `  - ${text}\n`;
          break;
        case 'pre':
          textContent += `\n\`\`\`\n${text}\n\`\`\`\n\n`;
          break;
        case 'blockquote':
          textContent += `\n> ${text}\n\n`;
          break;
        default:
          textContent += `${text}\n\n`;
      }
    });

    const blob = new Blob([textContent], { type: 'text/markdown;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `${title.replace(/[^a-z0-9]/gi, '-').toLowerCase()}-personalized.md`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
    setShowDownloadMenu(false);
  }, [user]);

  const handleDownloadPDF = useCallback(async () => {
    const contentElement = getContentElement();
    if (!contentElement) return;

    setIsProcessing(true);

    try {
      // Dynamically import html2pdf to avoid SSR issues
      const html2pdf = (await import('html2pdf.js')).default;

      const title = getChapterTitle();
      const date = new Date().toLocaleDateString();

      // Create a clone of the content for PDF generation
      const pdfContent = document.createElement('div');
      pdfContent.innerHTML = `
        <div style="font-family: Arial, sans-serif; padding: 20px;">
          <div style="text-align: center; margin-bottom: 30px; border-bottom: 2px solid #3b82f6; padding-bottom: 20px;">
            <h1 style="color: #1e40af; margin: 0;">${title}</h1>
            <p style="color: #6b7280; margin-top: 10px;">
              Downloaded on: ${date}<br/>
              Edited by: ${user?.name || user?.email || 'User'}
            </p>
          </div>
          <div style="line-height: 1.8;">
            ${contentElement.innerHTML}
          </div>
          <div style="margin-top: 40px; padding-top: 20px; border-top: 1px solid #e5e7eb; text-align: center; color: #9ca3af; font-size: 12px;">
            Physical AI & Humanoid Robotics Course - Personalized Content
          </div>
        </div>
      `;

      // Remove editable styles and contenteditable attributes from clone
      pdfContent.querySelectorAll('[contenteditable]').forEach((el) => {
        el.removeAttribute('contenteditable');
      });
      pdfContent.querySelectorAll(`.${styles.editable}`).forEach((el) => {
        el.classList.remove(styles.editable);
      });

      const options = {
        margin: [10, 10, 10, 10],
        filename: `${title.replace(/[^a-z0-9]/gi, '-').toLowerCase()}-personalized.pdf`,
        image: { type: 'jpeg', quality: 0.98 },
        html2canvas: {
          scale: 2,
          useCORS: true,
          logging: false,
        },
        jsPDF: {
          unit: 'mm',
          format: 'a4',
          orientation: 'portrait'
        },
        pagebreak: { mode: ['avoid-all', 'css', 'legacy'] }
      };

      await html2pdf().set(options).from(pdfContent).save();
    } catch (error) {
      console.error('PDF generation error:', error);
      alert('Failed to generate PDF. Please try again.');
    } finally {
      setIsProcessing(false);
      setShowDownloadMenu(false);
    }
  }, [user]);

  const handleRestore = useCallback(() => {
    const contentElement = getContentElement();
    if (!contentElement || !originalContent) return;

    const editableElements = contentElement.querySelectorAll('[contenteditable="true"]');
    editableElements.forEach((el) => {
      (el as HTMLElement).contentEditable = 'false';
      (el as HTMLElement).classList.remove(styles.editable);
    });

    contentElement.innerHTML = originalContent;
    setMode('original');
    setIsEditing(false);
  }, [originalContent]);

  // Only show for logged-in users
  if (!user) {
    return null;
  }

  return (
    <div className={styles.personalizeContainer} dir={currentLocale === 'ur' ? 'rtl' : 'ltr'}>
      <div className={styles.header}>
        <span className={styles.icon}>✨</span>
        <span className={styles.title}>{labels.personalizeContent}</span>
      </div>

      <div className={styles.buttonGroup}>
        <button
          className={`${styles.button} ${styles.simplifyButton} ${mode === 'simplified' ? styles.active : ''}`}
          onClick={handleSimplify}
          disabled={isProcessing || mode === 'simplified'}
        >
          {isProcessing && mode !== 'simplified' ? (
            <span className={styles.spinner}></span>
          ) : (
            <span className={styles.buttonIcon}>📖</span>
          )}
          {labels.simplify}
        </button>

        <button
          className={`${styles.button} ${styles.expandButton} ${mode === 'expanded' ? styles.active : ''}`}
          onClick={handleExpand}
          disabled={isProcessing || mode === 'expanded'}
        >
          {isProcessing && mode !== 'expanded' ? (
            <span className={styles.spinner}></span>
          ) : (
            <span className={styles.buttonIcon}>🔍</span>
          )}
          {labels.expand}
        </button>

        <button
          className={`${styles.button} ${styles.editButton} ${isEditing ? styles.active : ''}`}
          onClick={handleEdit}
          disabled={isProcessing}
        >
          <span className={styles.buttonIcon}>{isEditing ? '✓' : '✏️'}</span>
          {isEditing ? labels.doneEditing : labels.edit}
        </button>

        <div className={styles.downloadWrapper}>
          <button
            className={`${styles.button} ${styles.downloadButton}`}
            onClick={() => setShowDownloadMenu(!showDownloadMenu)}
            disabled={isProcessing}
          >
            {isProcessing ? (
              <span className={styles.spinner}></span>
            ) : (
              <span className={styles.buttonIcon}>💾</span>
            )}
            {labels.download}
            <span className={styles.dropdownArrow}>▼</span>
          </button>

          {showDownloadMenu && (
            <div className={styles.downloadMenu}>
              <button
                className={styles.downloadMenuItem}
                onClick={handleDownloadMD}
              >
                <span className={styles.menuIcon}>📄</span>
                {labels.downloadMD}
              </button>
              <button
                className={styles.downloadMenuItem}
                onClick={handleDownloadPDF}
              >
                <span className={styles.menuIcon}>📑</span>
                {labels.downloadPDF}
              </button>
            </div>
          )}
        </div>

        {(mode !== 'original' || originalContent) && (
          <button
            className={`${styles.button} ${styles.restoreButton}`}
            onClick={handleRestore}
            disabled={isProcessing}
          >
            <span className={styles.buttonIcon}>↩️</span>
            {labels.restore}
          </button>
        )}
      </div>

      {(mode !== 'original' || isEditing) && (
        <div className={styles.statusBadge}>
          {mode === 'simplified' && labels.simplifiedView}
          {mode === 'expanded' && labels.expandedView}
          {isEditing && labels.editMode}
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;
