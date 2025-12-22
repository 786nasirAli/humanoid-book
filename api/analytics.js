export default function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { event, data } = req.body;

  if (!event) {
    return res.status(400).json({ error: 'Event type is required' });
  }

  try {
    // In a production environment, you would store this data in a database
    // For now, we'll just log it
    console.log(`Analytics Event: ${event}`, {
      data,
      timestamp: new Date().toISOString()
    });

    res.status(200).json({ status: 'Analytics recorded' });
  } catch (error) {
    console.error('Analytics error:', error);
    res.status(500).json({ error: 'Failed to record analytics' });
  }
}