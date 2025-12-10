// src/components/Translation/UrduTranslation.js
import React, { useState, useEffect } from 'react';
import styles from './UrduTranslation.module.css';

const UrduTranslation = ({ contentId, originalContent }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Mock Urdu translations for common robotics/AI terms
  const technicalTerms = {
    'robot': 'روبوٹ',
    'humanoid': 'ہیومنوائڈ',
    'ROS 2': 'ROS 2',
    'Gazebo': 'گزیבו',
    'Unity': 'یونٹی',
    'NVIDIA Isaac': 'این وی ڈی آئیا ۔ اسحاق',
    'simulation': 'سمولیشن',
    'perception': 'ادراک',
    'navigation': 'رہ نما',
    'AI': 'مصنوعی ذہانت',
    'artificial intelligence': 'مصنوعی ذہانت',
    'machine learning': 'مشین لرننگ',
    'computer vision': 'کمپیوٹر وژن',
    'sensor': 'سینسر',
    'controller': 'کنٹرولر',
    'actuator': 'اکچوایٹر',
    'kinematics': 'کنیمیٹکس',
    'dynamics': 'ڈینامکس',
    'trajectory': 'مسیر',
    'path planning': 'راستہ منصوبہ بندی',
    'control system': 'کنٹرول سسٹم',
    'feedback': 'ردعمل',
    'algorithm': 'الگورتھم',
    'neural network': 'نیورل نیٹ ورک',
    'deep learning': 'گہری سیکھ',
    'reinforcement learning': 'تعزیز سیکھ',
    'slam': 'سلیم',
    'localization': 'مقام کار',
    'mapping': 'نقشہ سازی',
    'locomotion': 'چلنے کا طریقہ',
    'manipulation': 'ہیرا پھیری',
    'grasping': 'پکڑنا',
    'grasp': 'پکڑ',
    'manipulator': 'ہیرا پھیری والا',
    'end-effector': 'آخری ایفیکٹر',
    'gripper': 'گریپر',
    'mobile robot': 'موبائل روبوٹ',
    'autonomous': 'خود مختار',
    'autonomy': 'خود مختاری',
    'embodied AI': 'لائف AI',
    'physical AI': 'جسمانی AI',
    'human-robot interaction': 'انسان-روبوٹ تعامل',
    'HRI': 'HRI',
    'social robotics': 'سماجی روبوٹکس',
    'ethics': 'اخلاق',
    'robotics': 'روبوٹکس',
    'automation': 'خودکار',
    'mechatronics': 'میکیٹرونکس',
    'actuation': 'کارروائی',
    'sensing': 'احساس',
    'processing': 'پروسیسنگ',
    'execution': 'عمل',
    'middleware': 'مڈل ویئر',
    'node': 'نود',
    'topic': 'موضوع',
    'service': 'سروس',
    'action': 'کارروائی',
    'message': 'پیغام',
    'publisher': 'شائع کرنے والا',
    'subscriber': 'سبسکرائب کرنے والا',
    'command': 'کمانڈ',
    'status': 'حالت',
    'velocity': 'رفتار',
    'acceleration': 'تیزی',
    'position': 'پوزیشن',
    'orientation': 'سمت',
    'rotation': 'گردش',
    'translation': 'ترجمہ',
    'coordinate': ' coordination',
    'frame': 'فریم',
    'transform': 'ترامیل',
    'quaternion': 'کوواٹرینن',
    'euler angle': 'ایلر اینگل',
    'jacobian': 'جیکوبیان',
    'inverse kinematics': 'معکوس کنیمیٹکس',
    'forward kinematics': 'آگے کنیمیٹکس',
    'torque': 'ٹارک',
    'force': 'قوت',
    'impedance': 'روکاوٹ',
    'compliance': 'اِطاعت',
    'stiffness': 'سختی',
    'damping': 'ڈیمپنگ',
    'stability': 'مستحکم',
    'balance': 'تولید',
    'gait': 'چال',
    'stance': 'کھڑے ہونے کی حیثیت',
    'swing': 'جھولنا',
    'zero moment point': 'صفر لمحہ نقطہ',
    'ZMP': 'ZMP',
    'capture point': 'کیپچر پوائنٹ',
    'CP': 'CP',
    'centroidal momentum': 'سینٹروڈل مومنٹم',
    'whole body control': ' whole body control',
    'WBC': 'WBC',
    'operational space': 'آپریشنل خلا',
    'task space': 'کام کا خلا',
    'joint space': 'جوڑ کا خلا',
    'workspace': 'ورک سپیس',
    'configuration space': 'تشکیل کا خلا',
    'c-space': 'c-space',
    'visibility graph': 'دیکھنے کا گراف',
    'voronoi diagram': 'وورونوئی ڈائرام',
    'cell decomposition': 'سیل ڈیکومپوزیشن',
    'probabilistic roadmap': 'احتمالی ریل روڈ میپ',
    'PRM': 'PRM',
    'rapidly exploring random tree': 'تیزی سے استفسار کرنے والا بے ترتیب درخت',
    'RRT': 'RRT',
    'potential field': 'потенشل فیلڈ',
    'vector field histogram': 'ویکٹر فیلڈ ہسٹوگرام',
    'VFH': 'VFH',
    'dynamic window approach': 'متحرک ونڈو کا نمونہ',
    'DWA': 'DWA',
    'trajectory rollout': 'مسیر کا اسکرال',
    'model predictive control': 'نمونہ کے پیش گو کنترول',
    'MPC': 'MPC',
    'linear quadratic regulator': 'لکیری چوکور ریگولیٹر',
    'LQR': 'LQR',
    'proportional integral derivative': 'تناسب انضمام نتیجہ',
    'PID': 'PID',
    'Kalman filter': 'کلمن فلٹر',
    'particle filter': 'پارٹیکل فلٹر',
    'extended Kalman filter': 'توسیع شدہ کلمن فلٹر',
    'EKF': 'EKF',
    'unscented Kalman filter': 'بغیر سنت کلمن فلٹر',
    'UKF': 'UKF',
    'fastSLAM': 'FastSLAM',
    'graphSLAM': 'GraphSLAM',
    'monoSLAM': 'MonoSLAM',
    'orbSLAM': 'OrbSLAM',
    'LIO-SAM': 'LIO-SAM',
    'LeGO-LOAM': 'LeGO-LOAM',
    'HDL-Graph-SLAM': 'HDL-Graph-SLAM'
  };

  // Function to translate basic content to Urdu
  const translateToUrdu = (text) => {
    let translated = text;
    
    // Apply translations for technical terms
    for (const [english, urdu] of Object.entries(technicalTerms)) {
      // Case insensitive replacement
      const regex = new RegExp(english, 'gi');
      translated = translated.replace(regex, (match) => {
        // Preserve original case - if original was capitalized, keep capitalization
        if (match === match.toUpperCase()) {
          return urdu.toUpperCase();
        } else if (match === match.charAt(0).toUpperCase() + match.substr(1).toLowerCase()) {
          // Capitalized word
          return urdu.charAt(0).toUpperCase() + urdu.substr(1);
        } else {
          return urdu;
        }
      });
    }
    
    return translated;
  };

  const toggleTranslation = async () => {
    if (isTranslated) {
      // Switch back to original content
      setIsTranslated(false);
      setTranslatedContent('');
    } else {
      // Translate content
      setLoading(true);
      setError(null);
      
      try {
        // In a real implementation, this would call an API for professional translation
        // For now, we'll use the mock translation function
        const translated = translateToUrdu(originalContent);
        setTranslatedContent(translated);
        setIsTranslated(true);
      } catch (err) {
        setError('Translation failed. Please try again.');
        console.error('Translation error:', err);
      } finally {
        setLoading(false);
      }
    }
  };

  return (
    <div className={styles.translationContainer}>
      <button 
        className={`${styles.translateButton} ${isTranslated ? styles.active : ''}`}
        onClick={toggleTranslation}
        disabled={loading}
      >
        {loading ? (
          <span>ترجمہ ہو رہا ہے...</span>
        ) : isTranslated ? (
          <span>انگریزی دیکھیں</span>
        ) : (
          <span>اردو میں ترجمہ کریں</span>
        )}
      </button>
      
      {error && (
        <div className={styles.error}>
          {error}
        </div>
      )}
      
      {isTranslated && (
        <div className={styles.translatedContent} dir="rtl">
          {translatedContent || originalContent}
        </div>
      )}
      
      {isTranslated && (
        <div className={styles.translationNote}>
          <small>نوٹ: یہ ایک خودکار ترجمہ ہے۔ تکنیکی اصطلاحات کا ترجمہ درست ہونے کا یقین نہیں ہے۔</small>
        </div>
      )}
    </div>
  );
};

export default UrduTranslation;