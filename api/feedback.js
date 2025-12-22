export default function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { messageId, feedback, messageText } = req.body;

  if (!messageId || !feedback) {
    return res.status(400).json({ error: 'Message ID and feedback are required' });
  }

  try {
    // In a production environment, you would store feedback in a database
    // For now, we'll just log it
    console.log(`Feedback received:`, {
      messageId,
      feedback,
      messageText: messageText ? messageText.substring(0, 100) + '...' : 'N/A',
      timestamp: new Date().toISOString()
    });

    res.status(200).json({ status: 'Feedback recorded' });
  } catch (error) {
    console.error('Feedback error:', error);
    res.status(500).json({ error: 'Failed to record feedback' });
  }
}