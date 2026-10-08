export default function Banner({ message, onClose }: { message: string | null; onClose: () => void }) {
  if (!message) return null;
  return (
    <div className="banner" role="alert">
      <span>{message}</span>
      <button onClick={onClose}>Dismiss</button>
    </div>
  );
}
