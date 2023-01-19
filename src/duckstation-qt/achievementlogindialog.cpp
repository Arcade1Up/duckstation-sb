#include "achievementlogindialog.h"
#include "frontend-common/cheevos.h"
#include "qthostinterface.h"
#include <QtWidgets/QMessageBox>

AchievementLoginDialog::AchievementLoginDialog(QWidget* parent) : QDialog(parent)
{
  m_ui.setupUi(this);
  setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
  connectUi();
}

AchievementLoginDialog::~AchievementLoginDialog() = default;

void AchievementLoginDialog::loginClicked()
{
  const QString username(m_ui.userName->text());
  const QString password(m_ui.password->text());

  // TODO: Make cancellable.
  m_ui.status->setText(tr("Logging in..."));
  enableUI(false);

  QtHostInterface::GetInstance()->executeOnEmulationThread([this, username, password]() {
    const bool result = Cheevos::Login(username.toStdString().c_str(), password.toStdString().c_str());
    QMetaObject::invokeMethod(this, "processLoginResult", Qt::QueuedConnection, Q_ARG(bool, result));
  });
}

void AchievementLoginDialog::cancelClicked()
{
  done(1);
}

void AchievementLoginDialog::processLoginResult(bool result)
{
  if (!result)
  {
    QMessageBox::critical(this, tr("Login Error"),
                          tr("Login failed. Please check your username and password, and try again."));
    m_ui.status->setText(tr("Login failed."));
    enableUI(true);
    return;
  }

  done(0);
}

void AchievementLoginDialog::connectUi()
{
  connect(m_ui.login, &QPushButton::clicked, this, &AchievementLoginDialog::loginClicked);
  connect(m_ui.cancel, &QPushButton::clicked, this, &AchievementLoginDialog::cancelClicked);

  auto enableLoginButton = [this](const QString&) { m_ui.login->setEnabled(canEnableLoginButton()); };
  connect(m_ui.userName, &QLineEdit::textChanged, enableLoginButton);
  connect(m_ui.password, &QLineEdit::textChanged, enableLoginButton);
}

void AchievementLoginDialog::enableUI(bool enabled)
{
  m_ui.userName->setEnabled(enabled);
  m_ui.password->setEnabled(enabled);
  m_ui.cancel->setEnabled(enabled);
  m_ui.login->setEnabled(enabled && canEnableLoginButton());
}

bool AchievementLoginDialog::canEnableLoginButton() const
{
  return !m_ui.userName->text().isEmpty() && !m_ui.password->text().isEmpty();
}
